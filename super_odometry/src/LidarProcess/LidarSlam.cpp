
// LOCAL
#include "super_odometry/LidarProcess/LidarSlam.h"


//TODO: add to header file
double pose_parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> T_w_curr(pose_parameters);
Eigen::Map<Eigen::Quaterniond> Q_w_curr(pose_parameters + 3);

namespace super_odometry {


    LidarSLAM::LidarSLAM() {
        EdgesPoints.reset(new PointCloud());
        PlanarsPoints.reset(new PointCloud());
        WorldEdgesPoints.reset(new PointCloud());
        WorldPlanarsPoints.reset(new PointCloud());
    }
    void LidarSLAM::initROSInterface(rclcpp::Node::SharedPtr node) {
        node_ = node;
        pubUncertaintyX=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_X", 1);
        pubUncertaintyY=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_Y", 1);
        pubUncertaintyZ=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_Z", 1);
        pubUncertaintyRoll=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_roll", 1);
        pubUncertaintyPitch=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_pitch", 1);
        pubUncertaintyYaw=node_->create_publisher<std_msgs::msg::Float32>(ProjectName+"uncertainty_yaw", 1);
    }

    /**
     * [功能描述]：激光SLAM定位主函数
     * 该函数执行完整的定位和建图流程，包括状态初始化、点云处理、不确定性估计和位姿优化。
     * 
     * 工作流程：
     * 1. 使用当前位姿初始化系统状态
     * 2. 处理输入的角点和平面点云
     * 3. 根据是否为初始化阶段执行不同策略：
     *    - 初始化阶段：仅初始化地图，不进行优化
     *    - 正常运行：估计不确定性并执行定位建图优化
     * 
     * @param initialization - 是否为初始化阶段的标志（true=初始化，false=正常运行）
     * @param predictodom - 预测源类型（IMU/VIO/LIO等，用于设置初始位姿猜测）
     * @param position - 当前位姿的初始估计值（世界坐标系到激光雷达的变换矩阵）
     * @param edge_point - 角点（线特征）点云指针
     * @param planner_point - 平面点云指针
     * @param timeLaserOdometry - 当前帧激光里程计的时间戳
     * @return 无返回值，优化结果通过类成员变量存储
     */
    void LidarSLAM::Localization(
        bool initialization,
        PredictionSource predictodom,
        Transformd position,
        pcl::PointCloud<Point>::Ptr edge_point,
        pcl::PointCloud<Point>::Ptr planner_point,
        double timeLaserOdometry){  
       
       //! ========== 第一步：初始化系统状态 ==========
       // 使用当前位姿估计值初始化SLAM系统的状态变量
       // 包括位姿、速度、旋转矩阵等关键状态量
       initializeState(initialization, position);

       //! ========== 第二步：处理输入点云 ==========
       // 预处理输入的角点和平面点云
       // 注意：在优化步骤中，边缘点（edge points）会被移除，仅使用平面点进行配准
       processInputClouds(edge_point, planner_point);

       //! ========== 第三步：执行定位和建图 ==========
       // 根据初始化状态执行不同的处理策略
       if(!initialization){
        // 初始化阶段：仅初始化地图结构，建立初始局部地图
        // 此时不进行位姿优化，只是收集初始地图数据
        initializeMapping(timeLaserOdometry);
       } else{
        // 正常运行阶段：执行完整的定位和建图流程
        
        // 估计激光雷达测量的不确定性（协方差）
        // 用于后续优化中的权重分配和退化检测
        EstimateLidarUncertainty();
        
        // 执行定位和建图优化
        // 包括：点云配准、位姿优化、地图更新、退化检测等
        performLocalizationAndMapping(predictodom, timeLaserOdometry); 
       }
    }
     
    void LidarSLAM::initializeState(bool initialization, const Transformd&position){
        T_w_lidar=position;
        T_w_initial_guess=position;
        last_T_w_lidar=T_w_lidar;
    }
    

    void LidarSLAM::transformAndAddToMap(const pcl::PointCloud<Point>::Ptr&source_cloud, 
        pcl::PointCloud<Point>::Ptr&world_cloud, bool is_edge){

         //prepare point cloud 
         world_cloud->clear();
         world_cloud->points.reserve(source_cloud->size());
         world_cloud->header=source_cloud->header;

         //Transform points to world frame 
         for (const Point&p: *source_cloud){
            world_cloud->push_back(utils::TransformPointd(p,T_w_lidar));
         }

         //Add to local map 
         if(is_edge){
            localMap.addEdgePointCloud(*world_cloud);
         }else{
            localMap.addSurfPointCloud(*world_cloud);
         }

        }
    

    void LidarSLAM::initializeMapping(double timeLaserOdometry){
        //clear map and reset statistics 
        
        //set origin for local map 
        localMap.setOrigin(T_w_lidar.pos);

        //Transform and add feature points to map 
        transformAndAddToMap(EdgesPoints, WorldEdgesPoints, true);
        transformAndAddToMap(PlanarsPoints, WorldPlanarsPoints, false);

        lasttimeLaserOdometry=timeLaserOdometry;
    }
    

    void LidarSLAM::processInputClouds(const pcl::PointCloud<Point>::Ptr&edge_point, const pcl::PointCloud<Point>::Ptr&planner_point){
        //clear and reserve space for efficiency 
        EdgesPoints->clear();
        PlanarsPoints->clear();
        EdgesPoints->reserve(edge_point->size());
        PlanarsPoints->reserve(planner_point->size());
        *EdgesPoints=*edge_point;
        *PlanarsPoints=*planner_point;
    }    
    
     /**
      * [功能描述]：执行定位和建图的核心优化函数
      * 
      * 该函数实现基于ICP（迭代最近点）的激光SLAM优化算法，主要流程包括：
      * 1. 准备优化状态和参数
      * 2. 检查特征数量是否足够进行优化
      * 3. 迭代执行ICP优化：
      *    - 提取点云特征并建立对应关系
      *    - 构建非线性优化问题
      *    - 求解优化问题得到位姿更新
      *    - 记录迭代统计信息
      *    - 检查收敛条件
      * 4. 执行后处理（更新地图、发布结果等）
      * 
      * @param predictodom - 预测源类型（IMU/VIO/LIO等），用于设置优化初值和约束
      * @param timeLaserOdometry - 当前帧的时间戳
      * @return 无返回值，优化结果存储在T_w_lidar等成员变量中
      */
     void LidarSLAM::performLocalizationAndMapping(PredictionSource predictodom, double timeLaserOdometry)
    {  
        //! ========== 第一步：初始化优化状态 ==========
        // 准备优化所需的状态变量（位姿、速度、协方差等）
        prepareOptimizationState();

        //! ========== 第二步：检查特征充分性 ==========
        // 验证是否有足够的特征点进行可靠的优化
        // 特征不足时无法进行有效的点云配准
        if(!hasEnoughFeatures()){
            RCLCPP_WARN(node_->get_logger(), "Not enough features for optimization");
            return;
        }
        
        //! ========== 第三步：执行ICP迭代优化 ==========
        // 开始优化计时
        TicToc t_opt;
        
        // ICP迭代循环：逐步优化位姿估计直到收敛或达到最大迭代次数
        for (size_t icp_iter=0; icp_iter<LocalizationICPMaxIter; ++icp_iter){
            
            // 初始化特征计数器
            int edge_num=0;      // 边缘特征（角点/线特征）数量
            int planner_num=0;   // 平面特征数量
            
            // 重置距离参数（用于特征匹配的距离阈值等）
            ResetDistanceParameters();
            
            // 创建迭代统计消息，用于记录本次迭代的详细信息
            super_odometry_msgs::msg::IterationStats iter_stats;

            // 使用TBB并发容器存储特征对应关系，提高多线程性能
            tbb::concurrent_vector<OptimizationParameter> feature_corres;
            
            // 提取特征约束：在当前位姿估计下，为每个特征点寻找地图中的对应点
            // 建立点到线、点到面的几何约束关系
            extractFeaturesConstraints(feature_corres, edge_num, planner_num);
            
            //! ========== 第四步：设置并求解优化问题 ==========
            // 保存上一次迭代的位姿，用于计算位姿变化量和收敛判断
            Transformd previous_T(T_w_lidar);
          
            // 构建非线性优化问题（Ceres优化器）
            // 添加残差项：点到面距离、点到线距离、IMU约束、运动学约束等
            auto problem=setupOptimizationProblem(feature_corres, predictodom, T_w_initial_guess);
            
            // 求解优化问题，使用Levenberg-Marquardt算法最小化总残差
            // 返回优化摘要，包含收敛状态、残差变化、成功步数等信息
            auto summary=solveOptimizationProblem(problem);
            
            //! ========== 第五步：更新位姿估计 ==========
            // 将优化得到的位姿（T_w_curr和Q_w_curr）更新到系统状态
            T_w_lidar.pos=T_w_curr;  // 更新位置（平移向量）
            T_w_lidar.rot=Q_w_curr;  // 更新姿态（旋转四元数）
            
            //! ========== 第六步：记录迭代统计信息 ==========
            // 记录本次迭代的详细统计：特征数量、位姿变化、残差等
            // 用于性能分析和调试
            recordIterationStats(iter_stats, planner_num, edge_num, previous_T, T_w_lidar);
            
            //! ========== 第七步：检查收敛条件 ==========
            // 收敛判断条件：
            // 1. summary.num_successful_steps == 1: 优化器仅执行了一步就收敛（变化很小）
            // 2. icp_iter == LocalizationICPMaxIter - 1: 达到最大迭代次数
            if ((summary.num_successful_steps == 1) ||(icp_iter == this->LocalizationICPMaxIter - 1)) {
                // 估计配准误差（协方差），用于评估位姿估计的不确定性
                // 采样100个残差项来估计误差分布
                this->LocalizationUncertainty =
                        EstimateRegistrationError(problem, 100);
                break;  // 跳出ICP迭代循环
            }

      }
      
      //! ========== 第八步：后优化处理 ==========
      // 执行优化后的处理：更新局部地图、发布位姿结果、保存统计信息等
      performPostOptimizationProcessing(timeLaserOdometry, t_opt, stats);
    }

    
    void LidarSLAM::performPostOptimizationProcessing(double timeLaserOdometry, TicToc &t_opt, super_odometry_msgs::msg::OptimizationStats &stats) {
        // Apply manual yaw correction
        MannualYawCorrection();
        
        // Update statistics
        updateOptimizationStats(t_opt, stats);
        
        // Check motion thresholds and update map
        if (checkMotionThresholds(timeLaserOdometry, stats)) {
            // Transform and add new features to map
            transformAndAddToMap(EdgesPoints, WorldEdgesPoints, true);
            transformAndAddToMap(PlanarsPoints, WorldPlanarsPoints, false);
        }
        
        // Update timing
        lasttimeLaserOdometry = timeLaserOdometry;
    }

    bool LidarSLAM::checkMotionThresholds(double timeLaserOdometry, super_odometry_msgs::msg::OptimizationStats &stats) {
    
        bool acceptResult = true;
        double delta_t = timeLaserOdometry - lasttimeLaserOdometry;
        
        // Check velocity threshold
        if (stats.translation_from_last/delta_t > OptSet.velocity_failure_threshold) {
            T_w_lidar = last_T_w_lidar;
            startupCount = 5;
            acceptResult = false;
            RCLCPP_WARN(node_->get_logger(), "large motion detected, ignoring predictor for a while");
        }
        
        // Check small motion threshold
        if (stats.translation_from_last < 0.02 && stats.rotation_from_last < 0.005) {
            acceptResult = false;
            T_w_lidar = last_T_w_lidar;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "very small motion, not accumulating. %f", stats.translation_from_last);
        }
    acceptResult = true;
    return acceptResult;
}


    void LidarSLAM::updateOptimizationStats(TicToc &t_opt, super_odometry_msgs::msg::OptimizationStats &stats){
        double time_duration = t_opt.toc();
        stats.time_elapsed = time_duration;
        Transformd total_incremental_T;
        total_incremental_T = T_w_initial_guess.inverse() * T_w_lidar;
        stats.total_translation = (total_incremental_T).pos.norm();
        stats.total_rotation = 2 * atan2(total_incremental_T.rot.vec().norm(), total_incremental_T.rot.w());
        Transformd diff_from_last_T = last_T_w_lidar.inverse() * T_w_lidar;

        stats.translation_from_last = diff_from_last_T.pos.norm();
        stats.rotation_from_last = 2 * atan2(diff_from_last_T.rot.vec().norm(), diff_from_last_T.rot.w());
        last_T_w_lidar=T_w_lidar;
    }


    /**
     * [功能描述]：设置Ceres非线性优化问题
     * 
     * 该函数构建用于位姿估计的非线性最小二乘优化问题，包括：
     * 1. 创建Ceres优化问题实例
     * 2. 添加位姿参数块（使用自定义的局部参数化）
     * 3. 添加特征约束（点到线、点到面的距离残差）
     * 4. 根据预测源类型，条件性地添加绝对位姿约束
     * 
     * 优化目标：最小化所有残差项的加权平方和，得到最优位姿估计
     * 
     * @param features_corres - 特征对应关系容器，包含所有点到线/点到面的几何约束
     * @param predictsource - 预测源类型（IMU/VIO/LIO等），决定是否添加绝对位姿约束
     * @param position - 位姿初始猜测值，用于绝对位姿约束
     * @return 返回配置好的Ceres优化问题对象
     */
    ceres::Problem LidarSLAM::setupOptimizationProblem(const tbb::concurrent_vector<OptimizationParameter>&features_corres, 
                                                       PredictionSource predictsource, const Transformd&position){
        //! ========== 第一步：创建Ceres优化问题 ==========
        // 配置优化问题的选项（使用默认配置）
        ceres::Problem::Options problem_options; 
        // 创建优化问题实例
        ceres::Problem problem(problem_options);
        
        //! ========== 第二步：添加位姿参数块 ==========
        // 添加待优化的位姿参数块（7维：3维位置 + 4维四元数）
        // 使用自定义的PoseLocalParameterization进行参数化：
        // - 确保四元数归一化约束
        // - 使用李代数进行优化更新（避免奇异性）
        problem.AddParameterBlock(pose_parameters, 7, new PoseLocalParameterization());

        //! ========== 第三步：添加特征约束 ==========
        // 将所有特征对应关系转换为残差项并添加到优化问题中
        // 包括：点到线距离残差、点到面距离残差
        // 这些残差构成了位姿优化的主要约束
        addFeatureConstraints(problem, features_corres);

       
        //! ========== 第四步：条件性添加绝对位姿约束 ==========
        // 根据预测源类型判断是否需要添加绝对位姿约束
        // 绝对位姿约束的作用：
        // - 当特征不足或退化时，使用IMU/VIO预测值作为先验约束
        // - 防止优化结果偏离初始猜测过远
        // - 提高系统在特征缺乏场景下的鲁棒性
        if(shouldAddAbsolutePoseConstraints(predictsource)){
            // 添加绝对位姿约束（包含位置和姿态的先验残差）
            // features_corres.size()用于自适应调整约束权重
            addAbsolutePoseConstraints(problem,position, features_corres.size());
        }
        
        //! ========== 第五步：返回优化问题 ==========
        // 返回配置完成的优化问题，供求解器使用
        return problem;
    }

    ceres::Solver::Summary LidarSLAM::solveOptimizationProblem(ceres::Problem&problem){
        ceres::Solver::Options options;
        options.max_num_iterations=4;
        options.linear_solver_type=ceres::DENSE_QR;
        options.minimizer_progress_to_stdout=false;
        options.check_gradients=false;
        options.gradient_check_relative_precision=1e-4;
        ceres::Solver::Summary summary; 
        ceres::Solve(options, &problem, &summary);
        return summary;
    }

    void LidarSLAM::recordIterationStats(super_odometry_msgs::msg::IterationStats& iter_stats,
                                        int surf_num, int edge_num, Transformd&previous_T, Transformd&current_T){
        //Record iteration statistics 
        iter_stats.num_surf_from_scan=surf_num;
        iter_stats.num_corner_from_scan=edge_num;
        Transformd incremental_T=previous_T.inverse()*current_T;
        iter_stats.translation_norm=incremental_T.pos.norm();
        iter_stats.rotation_norm=2*atan2(incremental_T.rot.vec().norm(), incremental_T.rot.w());
        stats.iterations.push_back(iter_stats);
    }
    

    /**
     * [功能描述]：向优化问题添加特征约束（残差项）
     * 
     * 该函数将所有特征对应关系转换为Ceres优化问题的残差项，包括：
     * 1. 边缘特征：点到线的距离残差
     * 2. 平面特征：点到面的距离残差
     * 
     * 对每个残差项应用：
     * - Tukey损失函数：抑制外点影响，提高鲁棒性
     * - 权重缩放：根据特征可靠性自适应调整残差贡献
     * 
     * @param problem - Ceres优化问题对象（输入/输出）
     * @param features_corres - 特征对应关系容器，包含所有匹配成功的特征约束
     * @return 无返回值，残差项直接添加到problem中
     */
    void LidarSLAM::addFeatureConstraints(ceres::Problem&problem, const tbb::concurrent_vector<OptimizationParameter>&features_corres){
        //! ========== 初始化特征计数器 ==========
       int edge_num=0;      // 边缘特征计数
       int planner_num=0;   // 平面特征计数
       
        //! ========== 遍历所有特征约束 ==========
        for(const auto&constraint: features_corres){
            
            // 判断特征类型：边缘特征（角点/线特征）
            if(constraint.feature_type==FeatureType::EdgeFeature){
                //! ========== 处理边缘特征（点到线约束） ==========
                
                // 创建边缘特征的代价函数（解析形式）
                // 参数说明：
                // - Xvalue: 当前帧中的特征点坐标
                // - corres.first, corres.second: 局部地图中构成线特征的两个端点
                // 残差计算：点到线的垂直距离
                ceres::CostFunction*cost_function=new EdgeAnalyticCostFunction
                (constraint.Xvalue, constraint.corres.first, constraint.corres.second);
                
                // 使用Tukey鲁棒损失函数来限制外点的影响
                // 原理：对大残差进行削减，防止误匹配点主导优化过程
                // 阈值：sqrt(3*lineRes_)，与线特征分辨率相关
                auto *loss_function=new ceres::TukeyLoss(std::sqrt(3*localMap.lineRes_));
                
                // 使用缩放损失函数根据匹配可靠性加权残差
                // residualCoefficient：残差系数，反映特征匹配的置信度
                // 高置信度匹配获得更大权重，低置信度匹配权重较小
                // TAKE_OWNERSHIP：Ceres接管loss_function的内存管理
                auto *weight_function=new ceres::ScaledLoss(loss_function, constraint.residualCoefficient, ceres::TAKE_OWNERSHIP);
                
                // 将残差块添加到优化问题中
                // 参数：代价函数、损失函数、待优化的位姿参数
                problem.AddResidualBlock(cost_function, weight_function, pose_parameters);
                edge_num++;
                
            }else if(constraint.feature_type==FeatureType::PlaneFeature){
                //! ========== 处理平面特征（点到面约束） ==========
                
                // 创建平面特征的代价函数（解析形式）
                // 参数说明：
                // - Xvalue: 当前帧中的特征点坐标
                // - NormDir: 平面的法向量
                // - negative_OA_dot_norm: 平面方程的常数项（-d，其中平面方程为 n·x + d = 0）
                // 残差计算：点到平面的垂直距离（带符号）
                ceres::CostFunction*cost_function=new SurfNormAnalyticCostFunction(constraint.Xvalue, constraint.NormDir, constraint.negative_OA_dot_norm);
                
                // 使用Tukey鲁棒损失函数抑制平面匹配的外点
                // 阈值：sqrt(3*planeRes_)，与平面特征分辨率相关
                // 平面匹配通常比线匹配更稳定，但仍需鲁棒性处理
                auto *loss_function=new ceres::TukeyLoss(std::sqrt(3*localMap.planeRes_));
                
                // 根据平面匹配的可靠性加权残差
                // 平整度高、点数多的平面获得更高权重
                auto *weight_function=new ceres::ScaledLoss(loss_function, constraint.residualCoefficient, ceres::TAKE_OWNERSHIP);
                
                // 将平面残差块添加到优化问题中
                problem.AddResidualBlock(cost_function, weight_function, pose_parameters);
                planner_num++;
            }
        }
        
        // 设置预测源标志为0（表示使用特征约束）
        stats.prediction_source=0;
    }
    
    bool LidarSLAM::shouldAddAbsolutePoseConstraints(PredictionSource predictodom){
        return predictodom==PredictionSource::VIO_ODOM and isDegenerate==true and Visual_confidence_factor!=0;
    }

    void LidarSLAM::addAbsolutePoseConstraints(ceres::Problem&problem, const Transformd&position, int good_feature_num){
        //Add absolute pose constraint 
       Eigen::Matrix<double, 6, 6, Eigen::RowMajor> information;
       information.setIdentity();
       information(0, 0) =(1 - lidarOdomUncer.uncertainty_x) * std::max(50, int(good_feature_num*0.1))* Visual_confidence_factor;
       information(1, 1) =(1 - lidarOdomUncer.uncertainty_y) * std::max(50, int(good_feature_num*0.1))* Visual_confidence_factor;
       information(2, 2) =(1 - lidarOdomUncer.uncertainty_z) * std::max(50, int(good_feature_num*0.1))* Visual_confidence_factor;
       information(3, 3) = std::max(10, int(good_feature_num*0.01)) * Visual_confidence_factor;
       information(4, 4) = std::max(10, int(good_feature_num*0.01)) * Visual_confidence_factor;
       information(5, 5) = std::max(5, int(good_feature_num*0.001)) * 0;     
       SE3AbsolutatePoseFactor *absolutatePoseFactor=new SE3AbsolutatePoseFactor(position, information);
       problem.AddResidualBlock(absolutatePoseFactor, nullptr, pose_parameters);
       stats.prediction_source=1;
    }
    void LidarSLAM::extractFeaturesConstraints(
        tbb::concurrent_vector<LidarSLAM::OptimizationParameter>&feature_corres,
        int &edge_num, int &planner_num){

        //Process edge features 
        processEdgeFeatures(feature_corres, edge_num);

        //Process planner features 
        processPlannerFeatures(feature_corres,planner_num);
    }

    /**
     * [功能描述]：处理边缘特征（线特征/角点）并建立点到线的几何约束
     * 
     * 该函数遍历所有边缘特征点，为每个点在局部地图中寻找对应的线特征，
     * 建立点到线的距离约束，用于后续的位姿优化。
     * 
     * 工作流程：
     * 1. 检查边缘点云是否为空
     * 2. 遍历所有边缘特征点
     * 3. 为每个点计算与局部地图中线特征的对应关系和距离参数
     * 4. 筛选匹配成功的约束并统计匹配结果
     * 
     * 注意：边缘特征通常是场景中的角点或线条，如建筑物边缘、桌角等
     * 
     * @param features_corres - 输出参数，存储匹配成功的特征约束（点到线的几何关系）
     * @param edge_num - 输出参数，统计成功匹配的边缘特征数量
     * @return 无返回值，结果通过引用参数返回
     */
    void LidarSLAM::processEdgeFeatures(tbb::concurrent_vector<OptimizationParameter>&features_corres, int &edge_num){
        // 如果边缘点云为空，直接返回，无需处理
        if(EdgesPoints->empty()) return; 
        
        // 初始化边缘特征计数器
        edge_num=0;
        
        // 遍历所有边缘特征点
        for(const auto&p: *EdgesPoints){
            // 计算当前点与局部地图中线特征的对应关系
            // 在局部地图中搜索最近的线特征，计算点到线的距离参数
            // 返回的constraint包含：匹配结果、距离、线的方向向量等几何信息
            auto constraint=ComputeLineDistanceParameters(localMap, p);
            
            // 检查匹配是否成功
            if(constraint.match_result==MatchingResult::SUCCESS){
                // 匹配成功：将约束添加到特征对应关系容器中
                // 这些约束将用于构建优化问题的残差项
                features_corres.push_back(constraint);
                // 成功匹配的边缘特征计数加1
                edge_num++;
            }
            
            // 统计匹配结果到直方图中（成功/失败/距离过远等）
            // 用于分析匹配质量和调试
            MatchRejectionHistogramLine[constraint.match_result]++;
        }
    }

    /**
     * [功能描述]：处理平面特征并建立点到面的几何约束
     * 
     * 该函数遍历平面特征点，为每个点在局部地图中寻找对应的平面特征，
     * 建立点到面的距离约束，并更新可观测性直方图用于不确定性估计。
     * 
     * 工作流程：
     * 1. 检查平面点云是否为空
     * 2. 计算采样率（平面点通常很多，需要降采样以提高效率）
     * 3. 遍历平面特征点（根据采样率跳过部分点）
     * 4. 为每个点计算与局部地图中平面的对应关系和距离参数
     * 5. 筛选匹配成功的约束并更新可观测性直方图
     * 
     * 可观测性直方图：记录不同方向平面特征的分布，用于估计位姿不确定性
     * 
     * @param features_corres - 输出参数，存储匹配成功的特征约束（点到面的几何关系）
     * @param planner_num - 输出参数，统计成功匹配的平面特征数量
     * @return 无返回值，结果通过引用参数返回
     */
    void LidarSLAM::processPlannerFeatures(tbb::concurrent_vector<OptimizationParameter>&features_corres, int &planner_num){
        // 如果平面点云为空，直接返回，无需处理
        if(PlanarsPoints->empty()) return;
        
        //! ========== 第一步：计算采样率 ==========
        // 平面特征点数量通常远多于边缘特征
        // 为了平衡计算效率和精度，需要对平面点进行自适应降采样
        // 点数越多，采样率越低（跳过更多点）
        double sampling_rate=calculateSamplingRate(PlanarsPoints->size());
        
        // 初始化平面特征计数器
        planner_num=0;
        
        //! ========== 第二步：遍历平面特征点 ==========
        for(size_t i=0; i<PlanarsPoints->size(); ++i){
            // 根据采样率判断是否处理当前点
            // 跳过不需要处理的点，实现降采样
            if(!shouldProcessPoint(i,sampling_rate)) continue;
            
            // 获取当前平面特征点的引用
            const Point&p=PlanarsPoints->points[i];
            
            //! ========== 第三步：计算平面约束 ==========
            // 在局部地图中搜索最近的平面特征，计算点到平面的距离参数
            // 返回的constraint包含：
            // - 匹配结果（成功/失败/距离过远等）
            // - 平面法向量
            // - 点到平面的距离
            // - 平面的可观测性信息（用于不确定性估计）
            auto constraint=ComputePlaneDistanceParameters(localMap, p);
            
            //! ========== 第四步：处理匹配成功的约束 ==========
            if(constraint.match_result==MatchingResult::SUCCESS){
                // 匹配成功：将约束添加到特征对应关系容器中
                // 这些约束将用于构建优化问题的点到面残差项
                features_corres.push_back(constraint);
                
                // 成功匹配的平面特征计数加1
                planner_num++;
                
                //! ========== 第五步：更新可观测性直方图 ==========
                // 可观测性向量(observability)描述了平面法向量的方向分布
                // 用于评估该平面对不同自由度的约束能力：
                // - obs[0], obs[1]: 对roll/pitch角的观测索引
                // - obs[2]: 对平移的观测索引
                const auto&obs=constraint.feature.observability;
                
                // 更新直方图的三个bin，统计各方向平面特征的数量
                // 这些统计信息将用于EstimateLidarUncertainty()函数估计不确定性
                PlaneFeatureHistogramObs[obs[0]]++;  // 更新第一个观测维度的计数
                PlaneFeatureHistogramObs[obs[1]]++;  // 更新第二个观测维度的计数
                PlaneFeatureHistogramObs[obs[2]]++;  // 更新第三个观测维度的计数
            }
            
            //! ========== 第六步：统计匹配结果 ==========
            // 记录所有匹配尝试的结果（成功/失败/距离过远等）到直方图
            // 用于分析匹配质量、调试和性能评估
            MatchRejectionHistogramPlane[constraint.match_result]++;
        }
       
    } 

    double LidarSLAM::calculateSamplingRate(size_t num_points){
        if(num_points>OptSet.max_surface_features){   
            return 1.0*OptSet.max_surface_features/num_points;
        }
        return -1.0;
    }

    bool LidarSLAM::shouldProcessPoint(size_t index, double sampling_rate){
        if(sampling_rate<0.0) return true;
        double remainder = fmod(index*sampling_rate, 1.0);
        if (remainder + 0.001 > sampling_rate)
            return false;
        return true;
    }

    void LidarSLAM::prepareOptimizationState(){
        
        pos_in_localmap=localMap.shiftMap(T_w_lidar.pos); 
        T_w_curr=T_w_lidar.pos;
        Q_w_curr=T_w_lidar.rot; 

        auto [edge_count, planner_count]=localMap.get5x5LocalMapFeatureSize(pos_in_localmap);
        updateFeatureStats(edge_count, planner_count);
    } 

    void LidarSLAM::updateFeatureStats(size_t edge_count, size_t planner_count){
        stats.laser_cloud_corner_from_map_num = edge_count;
        stats.laser_cloud_surf_from_map_num = planner_count;
        stats.laser_cloud_corner_stack_num = EdgesPoints->size();
        stats.laser_cloud_surf_stack_num = PlanarsPoints->size();
        stats.iterations.clear();
    }
    
    bool LidarSLAM::hasEnoughFeatures(){
         return stats.laser_cloud_surf_from_map_num>50;
    }
    void LidarSLAM::ComputePointInitAndFinalPose(
            LidarSLAM::MatchingMode matchingMode, const LidarSLAM::Point &p,
            Eigen::Vector3d &pInit, Eigen::Vector3d &pFinal) {
      
        const bool is_local_lization_step =
                matchingMode == MatchingMode::LOCALIZATION;
        const Eigen::Vector3d pos = p.getVector3fMap().cast<double>();

        if (this->Undistortion == UndistortionMode::OPTIMIZED and
            is_local_lization_step) {

        } else if (this->Undistortion == UndistortionMode::APPROXIMATED and
                   is_local_lization_step) {

        } else {
            pInit = pos;
            pFinal = this->T_w_lidar * pos;
        }
    }

LidarSLAM::OptimizationParameter LidarSLAM::ComputeLineDistanceParameters(
            LocalMap &local_map, const LidarSLAM::Point &p) {
    // 1. Initialize point
    Eigen::Vector3d pInit, pFinal;
    OptimizationParameter result;
    Eigen::Vector3d mean;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    if (!initializeAndTransformPoint(p, pInit, pFinal)) {
        result.match_result = MatchingResult::INVAVLID_NUMERICAL;
        return result;
    }

    // 2. Find neighbors using line-specific search
    std::vector<Point> nearest_pts;
    std::vector<float> nearest_dist;
    Point query{pFinal.x(), pFinal.y(), pFinal.z()};
    bool found = local_map.nearestKSearchSpecificEdgePoint(
                query, nearest_pts, nearest_dist, LocalizationLineDistanceNbrNeighbors,
                static_cast<float>(this->LocalizationLineMaxDistInlier));
 
    if (!validateNeighborSearch(found, nearest_pts, nearest_dist, result)) {
        return result;
    }

    // 3. Compute and validate PCA
    if (!computePCAForFeature(nearest_pts, mean, eigenvalues, eigenvectors, result, FeatureType::EdgeFeature)) {
        return result;
    }

    // 4. Process results using shared components
    result=processLineResults(pInit, mean, eigenvalues, eigenvectors, nearest_pts, 3*local_map.lineRes_);
    return result;
}


LidarSLAM::OptimizationParameter LidarSLAM::processLineResults(
                                  const Eigen::Vector3d &pInit,
                                  const Eigen::Vector3d &mean,
                                  const Eigen::Vector3d &eigenvalues,
                                  const Eigen::Matrix3d &eigenvectors,
                                  const std::vector<Point> &nearest_pts,
                                  double square_max_dist) {
    OptimizationParameter result;
  // 1. Get line direction (principal component)
    Eigen::Vector3d line_direction = eigenvectors.col(2);
    line_direction.normalize();

    // 2. Compute projection matrix for point-to-line distance
    Eigen::Matrix3d projection_matrix = Eigen::Matrix3d::Identity() - 
    line_direction * line_direction.transpose();
    
    // 3. Validate projection matrix
    if (!projection_matrix.allFinite()) {
        result.match_result = MatchingResult::INVAVLID_NUMERICAL;
        return result;
    }

    // 4. Compute quality metrics
    double meanSquareDist = 0.0;
    for (const auto &pt : nearest_pts) {
        Eigen::Vector3d point_vec(pt.x, pt.y, pt.z);
        double squareDist = (point_vec - mean).transpose() * 
                           projection_matrix * (point_vec - mean);

        if (squareDist > 3*localMap.lineRes_) {
            result.match_result = MatchingResult::MSE_TOO_LARGE;
            return result;
        }
        meanSquareDist += squareDist;
    }
    meanSquareDist /= static_cast<double>(nearest_pts.size());

    // 5. Compute quality coefficient
    double fitQualityCoeff = 1.0 - std::sqrt(meanSquareDist / (3*localMap.lineRes_));

    // 6. Compute line endpoints for correspondence
    const double line_segment_length = 0.1; // 10cm line segment
    Eigen::Vector3d point_a = line_segment_length * line_direction + mean;
    Eigen::Vector3d point_b = -line_segment_length * line_direction + mean;

    // 7. Set result parameters
    result.feature_type = FeatureType::EdgeFeature;
    result.match_result = MatchingResult::SUCCESS;
    result.Avalue = projection_matrix;
    result.Pvalue = mean;
    result.Xvalue = pInit;
    result.corres = std::make_pair(point_a, point_b);
    result.TimeValue = 1.0;  // TODO: should be point cloud time
    result.residualCoefficient = fitQualityCoeff;
    return result;
}   

bool LidarSLAM::validateNeighborSearch(
        bool found,
        const std::vector<Point> &nearest_pts,
        const std::vector<float> &nearest_dist,
        OptimizationParameter &result) {
    
    if (!found || nearest_pts.size() < LocalizationMinmumLineNeighborRejection) {
        result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
        return false;
    }

    if (nearest_dist.back() > 3*localMap.lineRes_) {
        result.match_result = MatchingResult::NEIGHBORS_TOO_FAR;
        return false;
    }

    return true;
}

/**
 * [功能描述]：计算点到平面的距离参数和几何约束
 * 
 * 该函数为给定的平面特征点在局部地图中寻找对应的平面，通过PCA主成分分析
 * 拟合平面并计算点到平面的几何关系，用于后续的位姿优化。
 * 
 * 算法流程：
 * 1. 将点从激光雷达坐标系转换到世界坐标系
 * 2. 在局部地图中搜索最近邻点
 * 3. 使用PCA拟合平面（最小特征值对应的特征向量为法向量）
 * 4. 计算平面质量指标（平整度、拟合误差等）
 * 5. 修正法向量方向（确保指向传感器）
 * 6. 分析特征的可观测性（对各自由度的约束能力）
 * 7. 计算拟合质量系数作为权重
 * 
 * @param local_map - 局部地图对象，包含平面特征的KD树
 * @param p - 当前帧中的平面特征点
 * @return OptimizationParameter - 包含平面约束的所有几何信息和匹配结果
 */
LidarSLAM::OptimizationParameter LidarSLAM::ComputePlaneDistanceParameters(
            LocalMap &local_map, const Point &p) {
        OptimizationParameter result;
        
    //! ========== 第一步：初始化并转换点坐标 ==========
    // pInit: 点在激光雷达坐标系下的坐标
    // pFinal: 点转换到世界坐标系后的坐标
    Eigen::Vector3d pInit, pFinal;
    if (!initializeAndTransformPoint(p, pInit, pFinal)) {
        // 转换失败（通常是数值无效），返回失败结果
        result.match_result = MatchingResult::INVAVLID_NUMERICAL;
        return result;
    }
    
    //! ========== 第二步：设置搜索参数 ==========
    // 设置最近邻搜索所需的点数（用于PCA拟合平面）
    const size_t requiredNearest = LocalizationPlaneDistanceNbrNeighbors;
    // 设置最大搜索距离（平方），超过此距离的点不考虑
    // 使用3倍平面分辨率作为阈值，平衡搜索范围和精度
    const double square_max_dist = 3 * local_map.planeRes_;

    //! ========== 第三步：查找最近邻点 ==========
    std::vector<Point> nearest_pts;    // 存储找到的最近邻点
    std::vector<float> nearest_dist;   // 存储对应的距离
    
    // 在局部地图中搜索最近邻点
    // 参数：局部地图、查询点、输出容器、所需点数、最小点数(5)、最大距离、结果
    if (!findNearestNeighbors(local_map, pFinal, nearest_pts, nearest_dist, 
                             requiredNearest, 5, square_max_dist, result)) {
        // 搜索失败（点数不足或距离过远），返回失败结果
        return result;
    }

    //! ========== 第四步：执行PCA主成分分析 ==========
    Eigen::Vector3d mean;              // 最近邻点的质心（平面上的一点）
    Eigen::Vector3d eigenvalues;       // 特征值（λ1 >= λ2 >= λ3）
    Eigen::Matrix3d eigenvectors;      // 特征向量矩阵（列向量为特征向量）
    Eigen::Vector3d plane_normal;      // 平面法向量
    double negative_OA_dot_norm;       // 平面方程常数项 -d（平面方程：n·x + d = 0）
    
    // 对最近邻点进行PCA分析，提取平面特征
    // 最小特征值对应的特征向量（eigenvectors.col(0)）即为平面法向量
    if (!computePCAForFeature(nearest_pts, mean, eigenvalues, eigenvectors, result, FeatureType::PlaneFeature)) {
        // PCA失败（数值问题或点分布不构成平面），返回失败结果
        return result;
    }

    //! ========== 第五步：验证并计算质量指标 ==========
    // 计算平面拟合的质量指标：
    // - 平均平方距离：各点到拟合平面的距离平方的均值
    // - 验证平面是否足够平整（平整度阈值检查）
    double meanSquareDist = computePlaneQualityMetrics(nearest_pts, plane_normal, 
                                                      negative_OA_dot_norm, result);
    if (result.match_result != MatchingResult::SUCCESS) {
        // 质量不达标（平面太粗糙或点分布不均匀），返回失败结果
        return result;
    }
    
    //! ========== 第六步：修正法向量方向 ==========
    // 确保平面法向量指向传感器位置，保持一致性
    Eigen::Vector3d correct_normal;
    Eigen::Vector3d curr_point(pFinal.x(), pFinal.y(), pFinal.z());  // 当前点位置
    Eigen::Vector3d viewpoint_direction = curr_point;  // 从原点（传感器）到当前点的向量
    Eigen::Vector3d normal=eigenvectors.col(0);  // PCA得到的法向量
    
    // 计算法向量与视线方向的点积
    double dot_product = viewpoint_direction.dot(normal);
    correct_normal=normal;
    
    // 如果点积为负，说明法向量背向传感器，需要翻转
    if (dot_product < 0)
        correct_normal = -correct_normal;

    //! ========== 第七步：计算特征可观测性 ==========
    // 分析平面特征对各个自由度（位置和姿态）的约束能力
    pcaFeature feature;
    FeatureObservabilityAnalysis(
                feature, pFinal, eigenvalues, correct_normal, eigenvectors.col(2));
    // 可观测性分析结果用于：
    // 1. 生成可观测性直方图（PlaneFeatureHistogramObs）
    // 2. 评估位姿不确定性
    // 3. 检测退化场景

    //! ========== 第八步：计算拟合质量系数 ==========
    // 质量系数 = 1 - sqrt(平均距离/最大距离)
    // 范围 [0, 1]，值越大表示拟合质量越好
    // 用作优化中该约束的权重
    double fitQualityCoeff = 1.0 - sqrt(meanSquareDist / square_max_dist);
    
    //! ========== 第九步：设置结果参数 ==========
    // 将所有计算结果封装到OptimizationParameter中
    // 包括：平面质心、点坐标、法向量、平面方程常数、可观测性、质量系数等
    setPlaneResults(result, mean, pInit, plane_normal, negative_OA_dot_norm, feature, fitQualityCoeff);
    return result;
}

void LidarSLAM::FeatureObservabilityAnalysis(pcaFeature &feature, const Eigen::Vector3d &pFinal, 
                                          const Eigen::Vector3d &eigenvalues, 
                                          const Eigen::Vector3d &normal_direction, 
                                          const Eigen::Vector3d &principal_direction) {


    // 1. Initialize feature point and directions
    feature.pt.x = pFinal.x();
    feature.pt.y = pFinal.y();
    feature.pt.z = pFinal.z();
    normal_direction.normalized();
    principal_direction.normalized();
    feature.vectors.principalDirection = principal_direction.cast<float>();
    feature.vectors.normalDirection = normal_direction.cast<float>();
    
    // 2. Compute eigenvalues and geometric properties
    computeEigenProperties(feature, eigenvalues);
    
    // 3. Compute rotation axes and cross products
    auto rotated_axes = computeRotatedAxes();
    computeCrossProducts(feature, rotated_axes);
    
    // 4. Compute translation observability
    computeTranslationObservability(feature, rotated_axes);
    
    // 5. Analyze feature quality and observability
    analyzeFeatureObservability(feature);
}

void LidarSLAM::computeEigenProperties(pcaFeature &feature, const Eigen::Vector3d &eigenvalues) {
    // Compute square roots of eigenvalues
    feature.values.lamada1 = std::sqrt(eigenvalues(2));
    feature.values.lamada2 = std::sqrt(eigenvalues(1));
    feature.values.lamada3 = std::sqrt(eigenvalues(0));
    
    double sum_lamada = feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3;
    
    // Compute geometric properties
    if (sum_lamada == 0) {
        feature.curvature = 0;
    } else {
        feature.curvature = feature.values.lamada3 / sum_lamada;
    }
    
    feature.linear_2 = (feature.values.lamada1 - feature.values.lamada2) / feature.values.lamada1;
    feature.planar_2 = (feature.values.lamada2 - feature.values.lamada3) / feature.values.lamada1;
    feature.spherical_2 = feature.values.lamada3 / feature.values.lamada1;
}


LidarSLAM::RotatedAxes LidarSLAM::computeRotatedAxes() {
    const Eigen::Vector3f x_axis(1, 0, 0);
    const Eigen::Vector3f y_axis(0, 1, 0);
    const Eigen::Vector3f z_axis(0, 0, 1);
    
    Eigen::Quaternionf rot(T_w_lidar.rot.w(), T_w_lidar.rot.x(),
                          T_w_lidar.rot.y(), T_w_lidar.rot.z());
    rot.normalized();
    
    return RotatedAxes{
        rot * x_axis,
        rot * y_axis,
        rot * z_axis
    };
}

void LidarSLAM::computeTranslationObservability(
        pcaFeature &feature, 
        const RotatedAxes &axes) {
    
    float planar_squared = feature.planar_2 * feature.planar_2;
    
    feature.tx_dot = planar_squared * 
        std::abs(feature.vectors.normalDirection.dot(axes.x));
    feature.ty_dot = planar_squared * 
        std::abs(feature.vectors.normalDirection.dot(axes.y));
    feature.tz_dot = planar_squared * 
        std::abs(feature.vectors.normalDirection.dot(axes.z));
}

void LidarSLAM::analyzeFeatureObservability(pcaFeature &feature) {
    using QualityPair = std::pair<float, Feature_observability>;
    std::vector<QualityPair> rotation_quality = {
        {feature.rx_cross, Feature_observability::rx_cross},
        {feature.neg_rx_cross, Feature_observability::neg_rx_cross},
        {feature.ry_cross, Feature_observability::ry_cross},
        {feature.neg_ry_cross, Feature_observability::neg_ry_cross},
        {feature.rz_cross, Feature_observability::rz_cross},
        {feature.neg_rz_cross, Feature_observability::neg_rz_cross}
    };
    
    std::vector<QualityPair> trans_quality = {
        {feature.tx_dot, Feature_observability::tx_dot},
        {feature.ty_dot, Feature_observability::ty_dot},
        {feature.tz_dot, Feature_observability::tz_dot}
    };
    // Sort quality measures
    std::sort(rotation_quality.begin(), rotation_quality.end(), utils::compare_pair_first);
    std::sort(trans_quality.begin(), trans_quality.end(), utils::compare_pair_first);
    
    // Assign top observability measures
    feature.observability.at(0) = rotation_quality.at(0).second;
    feature.observability.at(1) = rotation_quality.at(1).second;
    feature.observability.at(2) = trans_quality.at(0).second;
    feature.observability.at(3) = trans_quality.at(1).second;
}


void LidarSLAM::computeCrossProducts(pcaFeature &feature, const RotatedAxes &axes) {
    Eigen::Vector3f point(feature.pt.x, feature.pt.y, feature.pt.z);
    Eigen::Vector3f cross = point.cross(feature.vectors.normalDirection);
    
    // Compute cross products with rotated axes
    feature.rx_cross = cross.dot(axes.x);
    feature.neg_rx_cross = -feature.rx_cross;
    feature.ry_cross = cross.dot(axes.y);
    feature.neg_ry_cross = -feature.ry_cross;
    feature.rz_cross = cross.dot(axes.z);
    feature.neg_rz_cross = -feature.rz_cross;
}

void LidarSLAM::setPlaneResults(OptimizationParameter &result, const Eigen::Vector3d &mean, 
                               const Eigen::Vector3d &pInit, const Eigen::Vector3d &plane_normal, 
                               double negative_OA_dot_norm, const pcaFeature &feature, double fitQualityCoeff) {

   result.feature_type = FeatureType::PlaneFeature;
   result.feature = feature;
   result.match_result = MatchingResult::SUCCESS;
   result.Pvalue = mean;
   result.Xvalue = pInit;
   result.NormDir = plane_normal;
   result.negative_OA_dot_norm = negative_OA_dot_norm;
   result.TimeValue =static_cast<double>(1.0);  // TODO:should be the point cloud time
   result.residualCoefficient = fitQualityCoeff;
}   




bool LidarSLAM::initializeAndTransformPoint(const Point &p, 
                                          Eigen::Vector3d &pInit,
                                          Eigen::Vector3d &pFinal) {
    ComputePointInitAndFinalPose(MatchingMode::LOCALIZATION, p, pInit, pFinal);
    return true;
}

bool LidarSLAM::findNearestNeighbors(LocalMap &local_map,
                                    const Eigen::Vector3d &pFinal,
                                    std::vector<Point> &nearest_pts,
                                    std::vector<float> &nearest_dist,
                                    size_t requiredNearest,
                                    size_t min_neighbors,
                                    double square_max_dist,
                                    OptimizationParameter &result) {
    Point pFinal_query;
    pFinal_query.x = pFinal.x();
    pFinal_query.y = pFinal.y();
    pFinal_query.z = pFinal.z();

    bool found = local_map.nearestKSearchSurf(pFinal_query, nearest_pts,
                                            nearest_dist, requiredNearest);

    if (!found || nearest_pts.size() < min_neighbors) {
        result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
        return false;
    }

    if (nearest_dist.back() > square_max_dist) {
        result.match_result = MatchingResult::NEIGHBORS_TOO_FAR;
        return false;
    }

    return true;
}

bool LidarSLAM::computePCAForFeature(const std::vector<Point> &nearest_pts,
                                  Eigen::Vector3d &mean,
                                  Eigen::Vector3d &eigenvalues,
                                  Eigen::Matrix3d &eigenvectors,
                                  OptimizationParameter &result,
                                  FeatureType feature_type) {

    Eigen::MatrixXd data(nearest_pts.size(), 3);
    for (size_t k = 0; k < nearest_pts.size(); k++) {
        const Point &pt = nearest_pts[k];
        data.row(k) << pt.x, pt.y, pt.z;
    }
    // 2. Compute PCA
    try {
        auto eig = utils::ComputePCA(data, mean);
        eigenvalues = eig.eigenvalues();
        eigenvectors = eig.eigenvectors();
    } catch (const std::exception& e) {
        result.match_result = MatchingResult::INVAVLID_NUMERICAL;
        return false;
    }
    
    if(feature_type == FeatureType::PlaneFeature){
        if (eigenvalues(0) < 1e-6 || eigenvalues(1) / eigenvalues(2) < 0.1) {
            result.match_result = MatchingResult::BAD_PCA_STRUCTURE;
            return false;
        }
    }else if(feature_type == FeatureType::EdgeFeature){
        if(!eigenvalues.allFinite())
        {
            result.match_result = MatchingResult::INVAVLID_NUMERICAL;
            return false;
        }
        
        if(eigenvalues(2) < LocalizationMinmumLineNeighborRejection * eigenvalues(1)){
            result.match_result = MatchingResult::BAD_PCA_STRUCTURE;
            return false;
        }

    }
    return true;
}

double LidarSLAM::computePlaneQualityMetrics(const std::vector<Point>& nearest_pts,
                                          Eigen::Vector3d &plane_normal,
                                          double &negative_OA_dot_norm,
                                          OptimizationParameter &result) {
    
     // 1. Set up the system of equations
    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    // 2. Fill matrix with point coordinates
    for (int i = 0; i < 5; i++) {
        matA0.row(i) << nearest_pts[i].x, nearest_pts[i].y, nearest_pts[i].z;
    }
    
    // 3. Solve for plane normal
    plane_normal = matA0.colPivHouseholderQr().solve(matB0);
    
    // 4. Check if solution is valid
    if (!plane_normal.allFinite()) {
        result.match_result = MatchingResult::INVAVLID_NUMERICAL;
        return 0.0;
    }
    
    // 5. Compute and store plane parameters
    negative_OA_dot_norm = 1.0 / plane_normal.norm();
    plane_normal.normalize();
    

    double meanSquareDist = 0.0;
    const double max_point_distance = localMap.planeRes_ / 2.0;
    
    // 1. Compute mean square distance to plane
    for (const auto& pt : nearest_pts) {
        double point_to_plane_dist = std::abs(
            plane_normal.x() * pt.x + 
            plane_normal.y() * pt.y +
            plane_normal.z() * pt.z + 
            negative_OA_dot_norm
        );
        
        // 2. Check if point is too far from plane
        if (point_to_plane_dist > max_point_distance) {
            result.match_result = MatchingResult::MSE_TOO_LARGE;
            return 0.0;
        }
        
        meanSquareDist += point_to_plane_dist;
    }
    
    // 3. Compute average distance
    meanSquareDist /= nearest_pts.size();
    result.match_result = MatchingResult::SUCCESS;
    return meanSquareDist;
}


    void LidarSLAM::ResetDistanceParameters() {
        this->OptimizationData.clear();
        for (auto &ele : MatchRejectionHistogramLine) ele = 0;
        for (auto &ele : MatchRejectionHistogramPlane) ele = 0;
        for (auto &ele : PlaneFeatureHistogramObs) ele = 0;
    }

    LidarSLAM::RegistrationError LidarSLAM::EstimateRegistrationError(
            ceres::Problem &problem, const double eigen_thresh) {
        RegistrationError err;

        // Covariance computation options
        ceres::Covariance::Options covOptions;
        covOptions.apply_loss_function = true;
        covOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
        covOptions.null_space_rank = -1;
        covOptions.num_threads = 2;

        ceres::Covariance covarianceSolver(covOptions);
        std::vector<std::pair<const double *, const double *>> covarianceBlocks;
        const double *paramBlock = pose_parameters;
        covarianceBlocks.emplace_back(paramBlock, paramBlock);
        covarianceSolver.Compute(covarianceBlocks, &problem);
        covarianceSolver.GetCovarianceBlockInTangentSpace(paramBlock, paramBlock,
                                                          err.Covariance.data());

        // Estimate max position/orientation errors and directions from covariance
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3, 3>());

        err.PositionError = std::sqrt(eigPosition.eigenvalues()(2));
        err.PositionErrorDirection = eigPosition.eigenvectors().col(2);
        err.PosInverseConditionNum = std::sqrt(eigPosition.eigenvalues()(0)) / std::sqrt(eigPosition.eigenvalues()(2));

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3, 3>());
        err.OrientationError = utils::Rad2Deg(std::sqrt(eigOrientation.eigenvalues()(2)));
        err.OrientationErrorDirection = eigOrientation.eigenvectors().col(2);
        err.OriInverseConditionNum =
                std::sqrt(eigOrientation.eigenvalues()(0)) / std::sqrt(eigOrientation.eigenvalues()(2));

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigPosition2(err.Covariance.inverse());

        return err;
    }
    
   void LidarSLAM::MannualYawCorrection()
   {
     
    Transformd last_current_T = last_T_w_lidar.inverse() * T_w_lidar;
    float translation_norm = last_current_T.pos.norm();

    double roll, pitch, yaw;
    tf2::Quaternion orientation(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(),
                                    T_w_lidar.rot.w());
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    
    tf2::Quaternion correct_orientation;

   
    double correct_yaw=yaw+translation_norm*OptSet.yaw_ratio*M_PI/180;
    correct_orientation.setRPY(roll, pitch, correct_yaw);
    
    Eigen::Quaterniond correct_rot;
    correct_rot= Eigen::Quaterniond(correct_orientation.w(), correct_orientation.x(), correct_orientation.y(),
                                correct_orientation.z());
    
    T_w_lidar.rot = correct_rot.normalized();
   }

   /**
    * [功能描述]：估计激光雷达里程计的测量不确定性
    * 
    * 该函数基于平面特征的几何分布直方图来估计6自由度位姿的不确定性。
    * 不确定性估计原理：
    * - 通过分析平面特征的法向量分布，评估各方向的约束强度
    * - 特征分布越均匀，约束越强，不确定性越低
    * - 特征分布越集中，约束越弱，不确定性越高（可能出现退化）
    * 
    * PlaneFeatureHistogramObs直方图索引说明（共9个bin）：
    * - 索引0-1: 对roll角敏感的平面特征（法向量在YZ平面附近）
    * - 索引2-3: 对pitch角敏感的平面特征（法向量在XZ平面附近）
    * - 索引4-5: 对yaw角敏感的平面特征（法向量在XY平面附近）
    * - 索引6: 对x平移敏感的平面特征（法向量接近X轴）
    * - 索引7: 对y平移敏感的平面特征（法向量接近Y轴）
    * - 索引8: 对z平移敏感的平面特征（法向量接近Z轴）
    * 
    * @return 无返回值，结果存储在lidarOdomUncer和stats成员变量中
    */
   void LidarSLAM::EstimateLidarUncertainty() {
        //! ========== 第一步：计算平移不确定性 ==========
        
        // 统计所有对平移敏感的平面特征总数
        // 索引6,7,8分别对应X,Y,Z方向的平面特征
        double TotalTransFeature = PlaneFeatureHistogramObs.at(6) +
                                   PlaneFeatureHistogramObs.at(7) +
                                   PlaneFeatureHistogramObs.at(8);
        
        // 计算X方向的不确定性
        // 原理：X方向平面特征占比越高，说明其他方向特征越少，X方向约束越弱，不确定性越高
        // 乘以3是缩放因子，使不确定性值在合理范围内
        double uncertaintyX = (PlaneFeatureHistogramObs.at(6) / TotalTransFeature) * 3;
        lidarOdomUncer.uncertainty_x = std::min(uncertaintyX, 1.0);  // 限制最大值为1.0

        // 计算Y方向的不确定性
        // 同样的原理：Y方向特征占比越高，Y方向约束越弱
        double uncertaintyY = (PlaneFeatureHistogramObs.at(7) / TotalTransFeature) * 3;
        lidarOdomUncer.uncertainty_y = std::min(uncertaintyY, 1.0);

        // 计算Z方向的不确定性
        // Z方向特征占比越高（通常是地面），其他方向约束越弱
        double uncertaintyZ = (PlaneFeatureHistogramObs.at(8) / TotalTransFeature) * 3;
        lidarOdomUncer.uncertainty_z = std::min(uncertaintyZ, 1.0);

        //! ========== 第二步：计算旋转不确定性 ==========
        
        // 统计所有对旋转敏感的平面特征总数
        // 索引0-5对应不同方向的平面，这些平面对旋转角度提供约束
        double TotalRotationFeature = PlaneFeatureHistogramObs.at(0) +
                                      PlaneFeatureHistogramObs.at(1) +
                                      PlaneFeatureHistogramObs.at(2) +
                                      PlaneFeatureHistogramObs.at(3) +
                                      PlaneFeatureHistogramObs.at(4) +
                                      PlaneFeatureHistogramObs.at(5);

        // 计算横滚角(roll)的不确定性
        // 索引0和1的特征对roll角敏感，占比越高说明roll方向退化
        double uncertaintyRoll =
                (PlaneFeatureHistogramObs.at(0) + PlaneFeatureHistogramObs.at(1)) / TotalRotationFeature * 3;
        lidarOdomUncer.uncertainty_roll = std::min(uncertaintyRoll, 1.0);

        // 计算俯仰角(pitch)的不确定性
        // 索引2和3的特征对pitch角敏感
        double uncertaintyPitch =
                (PlaneFeatureHistogramObs.at(2) + PlaneFeatureHistogramObs.at(3)) / TotalRotationFeature * 3;
        lidarOdomUncer.uncertainty_pitch = std::min(uncertaintyPitch, 1.0);

        // 计算偏航角(yaw)的不确定性
        // 索引4和5的特征对yaw角敏感（通常是竖直平面如墙壁）
        double uncertaintyYaw =
                (PlaneFeatureHistogramObs.at(4) + PlaneFeatureHistogramObs.at(5)) / TotalRotationFeature * 3;
        lidarOdomUncer.uncertainty_yaw = std::min(uncertaintyYaw, 1.0);

        //! ========== 第三步：处理无特征的情况 ==========      
        // 如果没有足够的平移或旋转特征，将所有不确定性设为0
        // 这种情况通常发生在特征提取失败或环境过于简单时
        if(TotalTransFeature==0 || TotalRotationFeature==0)
        {
          lidarOdomUncer.uncertainty_x=0;
          lidarOdomUncer.uncertainty_y=0;
          lidarOdomUncer.uncertainty_z=0;
          lidarOdomUncer.uncertainty_roll=0;
          lidarOdomUncer.uncertainty_pitch=0;
          lidarOdomUncer.uncertainty_yaw=0;   
        }

        //! ========== 第四步：发布不确定性信息 ==========
        // 通过ROS话题发布6自由度的不确定性，供其他模块使用（如传感器融合）
        publishUncertainty(lidarOdomUncer.uncertainty_x, lidarOdomUncer.uncertainty_y, lidarOdomUncer.uncertainty_z,
                                     lidarOdomUncer.uncertainty_roll, lidarOdomUncer.uncertainty_pitch, lidarOdomUncer.uncertainty_yaw);

        //! ========== 第五步：保存不确定性到统计信息 ==========
        // 将不确定性值存储到统计结构体中，用于后续分析和调试
        stats.uncertainty_x=lidarOdomUncer.uncertainty_x;
        stats.uncertainty_y=lidarOdomUncer.uncertainty_y;
        stats.uncertainty_z=lidarOdomUncer.uncertainty_z;
        stats.uncertainty_roll=lidarOdomUncer.uncertainty_roll;
        stats.uncertainty_pitch=lidarOdomUncer.uncertainty_pitch;
        stats.uncertainty_yaw=lidarOdomUncer.uncertainty_yaw;

        //! ========== 退化检测（已注释） ==========
        // 以下是基于不确定性阈值的退化检测逻辑（当前未启用）
        // 如果不确定性低于阈值或特征数量不足，则判定为退化场景
        // if (lidarOdomUncer.uncertainty_x<0.2 or lidarOdomUncer.uncertainty_y<0.1 or lidarOdomUncer.uncertainty_z<0.2) {
        //     isDegenerate = true;  // 平移不确定性过低，可能出现退化
        // }else if (PlaneFeatureHistogramObs.at(6)<20 or PlaneFeatureHistogramObs.at(7)<10 or PlaneFeatureHistogramObs.at(8)<10)
        // {
        //     isDegenerate = true;  // 特征数量不足，判定为退化
        // }
        // else {
        //     isDegenerate = false;
        // }
    }

    void LidarSLAM::publishUncertainty(double uncer_x, double uncer_y, double uncer_z,
        double uncer_roll, double uncer_pitch, double uncer_yaw)
    {

        std_msgs::msg::Float32 uncertainty_x;
        uncertainty_x.data = uncer_x;
        pubUncertaintyX->publish(uncertainty_x);

        std_msgs::msg::Float32 uncertainty_y;
        uncertainty_y.data = uncer_y;
        pubUncertaintyY->publish(uncertainty_y);

        std_msgs::msg::Float32 uncertainty_z;
        uncertainty_z.data = uncer_z;
        pubUncertaintyZ->publish(uncertainty_z);

        std_msgs::msg::Float32 uncertainty_roll;
        uncertainty_roll.data = uncer_roll;
        pubUncertaintyRoll->publish(uncertainty_roll);

        std_msgs::msg::Float32 uncertainty_pitch;
        uncertainty_pitch.data = uncer_pitch;
        pubUncertaintyPitch->publish(uncertainty_pitch);

        std_msgs::msg::Float32 uncertainty_yaw;
        uncertainty_yaw.data = uncer_yaw;
        pubUncertaintyYaw->publish(uncertainty_yaw);

    };

} /* super_odometry */
