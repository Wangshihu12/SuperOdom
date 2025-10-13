//
// Created by shiboz on 2021-10-18.
//

#include "super_odometry/LaserMapping/laserMapping.h"

double parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters);
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters+3);

Eigen::Vector3d vel_b;
Eigen::Vector3d ang_vel_b;

namespace super_odometry {

    laserMapping::laserMapping(const rclcpp::NodeOptions & options)
    : Node("laser_mapping_node", options) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }

    /**
     * [功能描述]：初始化激光建图模块的ROS2接口
     * 该函数负责：
     * 1. 配置ROS2回调组和订阅选项
     * 2. 读取全局参数、系统参数和标定参数
     * 3. 初始化点云降采样滤波器
     * 4. 创建话题订阅者和发布者
     * 5. 配置SLAM系统参数
     * 6. 启动定时处理任务
     * @return 无返回值
     */
    void laserMapping::initInterface() {
        //! ========== 第一步：配置ROS2回调组 ==========
        // 创建可重入回调组，允许多个回调并发执行
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        //! ========== 第二步：读取系统配置参数 ==========
        // 读取全局参数配置
        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read calibration. Exiting...");
            rclcpp::shutdown();
        }

        // 读取激光建图模块的参数配置
        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        // 读取传感器标定参数
        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        //! ========== 第三步：打印关键配置信息 ==========
        RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.debug_view_enabled);
        RCLCPP_INFO(this->get_logger(), "ENABLE OUSTER DATA: %d", config_.enable_ouster_data);
        RCLCPP_INFO(this->get_logger(), "line resolution %f plane resolution %f vision_laser_time_offset %f",
                config_.lineRes, config_.planeRes, vision_laser_time_offset);

        //! ========== 第四步：配置点云降采样滤波器 ==========
        // 设置角点（线特征）点云降采样的体素大小
        downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
        // 设置平面点云降采样的体素大小
        downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);


        //! ========== 第五步：创建话题订阅者 ==========
        // 订阅激光特征信息话题，接收特征提取节点发布的点云特征
        subLaserFeatureInfo = this->create_subscription<super_odometry_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2,
            std::bind(&laserMapping::laserFeatureInfoHandler, this,
                        std::placeholders::_1), sub_options);
                        

        //! ========== 第六步：创建点云发布者 ==========
        // 发布周围局部地图点云
        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_surround", 2);

        // 发布当前地图点云
        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_map", 2);

        // 发布全局地图点云
        pubLaserCloudPrior = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/overall_map", 2);

        // 发布配准后的全分辨率点云
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/registered_scan", 2);


        //! ========== 第七步：创建里程计发布者 ==========
        // 发布激光里程计结果
        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 1);

        // 发布增量式里程计（相对于初始位姿的增量）
        pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/aft_mapped_to_init_incremental", 1);


        // 发布视觉惯性里程计（VIO）预测结果
        pubVIOPrediction=  this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/vio_prediction", 1);

        // 发布激光惯性里程计（LIO）预测结果
        pubLIOPrediction= this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/lio_prediction", 1);


        //! ========== 第八步：创建路径和统计信息发布者 ==========
        // 发布激光里程计轨迹路径
        pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/laser_odom_path", 1);

        // 发布优化统计信息（迭代次数、残差等）
        pubOptimizationStats = this->create_publisher<super_odometry_msgs::msg::OptimizationStats>(
            ProjectName+"/super_odometry_stats", 1);

  
        // 发布预测源信息（IMU/VIO/LIO等）
        pubprediction_source = this->create_publisher<std_msgs::msg::String>(
            ProjectName+"/prediction_source", 1);

        //! ========== 第九步：创建定时处理任务 ==========
        // 创建100ms周期的定时器，用于周期性处理建图任务
        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(100.)),
            std::bind(&laserMapping::process, this));

        //! ========== 第十步：配置SLAM系统参数 ==========
        // 初始化SLAM的ROS接口
        slam.initROSInterface(shared_from_this());
        // 设置局部地图的线特征分辨率
        slam.localMap.lineRes_ = config_.lineRes;
        // 设置局部地图的平面特征分辨率
        slam.localMap.planeRes_ = config_.planeRes;
        // 设置视觉置信度因子，用于融合视觉信息时的权重
        slam.Visual_confidence_factor=config_.visual_confidence_factor;
        // 设置位置退化阈值，用于检测定位退化
        slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
        // 设置姿态退化阈值，用于检测姿态估计退化
        slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
        // 设置ICP最大迭代次数
        slam.LocalizationICPMaxIter=config_.max_iterations;
        // 启用/禁用调试可视化
        slam.OptSet.debug_view_enabled=config_.debug_view_enabled;
        // 设置速度失败阈值，用于检测运动异常
        slam.OptSet.velocity_failure_threshold=config_.velocity_failure_threshold;
        // 设置最大平面特征数量
        slam.OptSet.max_surface_features=config_.max_surface_features;
        // 设置偏航角权重比例
        slam.OptSet.yaw_ratio=yaw_ratio;
        // 设置地图保存目录
        slam.map_dir=config_.map_dir;
        // 设置定位模式（建图/定位）
        slam.localization_mode=config_.localization_mode;
        // 设置初始位置 (x, y, z)
        slam.init_x=config_.init_x;
        slam.init_y=config_.init_y;
        slam.init_z=config_.init_z;
        // 设置初始姿态 (roll, pitch, yaw)
        slam.init_roll=config_.init_roll;
        slam.init_pitch=config_.init_pitch;
        slam.init_yaw=config_.init_yaw;

        //! ========== 第十一步：初始化预测源和时间戳 ==========
        // 设置默认预测源为IMU方向
        prediction_source = PredictionSource::IMU_ORIENTATION;
        // 初始化IMU里程计时间戳为0
        timeLatestImuOdometry = rclcpp::Time(0,0,RCL_ROS_TIME);

        //! ========== 第十二步：初始化其他参数 ==========
        initializationParam();

    }

    void laserMapping::initializationParam() {

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());

        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes_rot.reset(new pcl::PointCloud<PointType>());
        laserCloudRawRes.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
        laserCloudRealsense.reset(new pcl::PointCloud<PointType>());
        laserCloudPriorOrg.reset(new pcl::PointCloud<PointType>());
        laserCloudPrior.reset(new pcl::PointCloud<PointType>());

        Eigen::Quaterniond q_wmap_wodom_(1, 0, 0, 0);
        Eigen::Vector3d t_wmap_wodom_(0, 0, 0);
        Eigen::Quaterniond q_wodom_curr_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_curr_(0, 0, 0);
        Eigen::Quaterniond q_wodom_pre_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_pre_(0, 0, 0);

        q_wmap_wodom = q_wmap_wodom_;
        t_wmap_wodom = t_wmap_wodom_;
        q_wodom_curr = q_wodom_curr_;
        t_wodom_curr = t_wodom_curr_;
        q_wodom_pre = q_wodom_pre_;
        t_wodom_pre = t_wodom_pre_;

        imu_odom_buf.allocate(5000);
        visual_odom_buf.allocate(5000);
        
        slam.localMap.setOrigin(Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z));

        if (slam.localization_mode) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m Loading GT Map now.... Please wait for 10 sec before running rosbag.\033[0m");
            if(utils::readPointCloud(config_.map_dir, laserCloudPrior)) {
                slam.localMap.addSurfPointCloud(*laserCloudPrior);
                pcl::toROSMsg(*laserCloudPrior, priorCloudMsg);
                priorCloudMsg.header.frame_id = WORLD_FRAME;
                RCLCPP_INFO(this->get_logger(), "\033[1;32m Loading GT Map Succesfully. Localization mode is Ready.\033[0m");
            } else {
                slam.localization_mode = false;
                RCLCPP_INFO(this->get_logger(), "\033[1;32mCannot read map file, switch to mapping mode.\033[0m");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mStart SLAM in mapping mode.\033[0m");
        }
    }

    bool laserMapping::readParameters()
    {
        // Declare with default values
        this->declare_parameter("laser_mapping_node.mapping_line_resolution", 0.1);
        this->declare_parameter("laser_mapping_node.mapping_plane_resolution", 0.2);
        this->declare_parameter("laser_mapping_node.max_iterations", 4);
        this->declare_parameter("laser_mapping_node.debug_view", false);
        this->declare_parameter("laser_mapping_node.enable_ouster_data", false);
        this->declare_parameter("laser_mapping_node.publish_only_feature_points", false);
        this->declare_parameter("laser_mapping_node.use_imu_roll_pitch", false);
        this->declare_parameter("laser_mapping_node.max_surface_features", 2000);
        this->declare_parameter("laser_mapping_node.velocity_failure_threshold", 30.0);
        this->declare_parameter("laser_mapping_node.auto_voxel_size", true);
        this->declare_parameter("laser_mapping_node.forget_far_chunks", false);
        this->declare_parameter("laser_mapping_node.visual_confidence_factor", 1.0);
        this->declare_parameter("laser_mapping_node.localization_mode", false); // Add default value!
        this->declare_parameter("laser_mapping_node.read_pose_file", false);
        this->declare_parameter("laser_mapping_node.init_x", 0.0);
        this->declare_parameter("laser_mapping_node.init_y", 0.0);
        this->declare_parameter("laser_mapping_node.init_z", 0.0);
        this->declare_parameter("laser_mapping_node.init_roll", 0.0);
        this->declare_parameter("laser_mapping_node.init_pitch", 0.0);
        this->declare_parameter("laser_mapping_node.init_yaw", 0.0);
        this->declare_parameter("map_dir", "pointcloud_local.pcd");


        // Get parameters
        config_.lineRes = this->get_parameter("laser_mapping_node.mapping_line_resolution").as_double();
        config_.planeRes = this->get_parameter("laser_mapping_node.mapping_plane_resolution").as_double();
        config_.max_iterations = this->get_parameter("laser_mapping_node.max_iterations").as_int();
        config_.debug_view_enabled = this->get_parameter("laser_mapping_node.debug_view").as_bool();
        config_.enable_ouster_data = this->get_parameter("laser_mapping_node.enable_ouster_data").as_bool();
        config_.publish_only_feature_points = this->get_parameter("laser_mapping_node.publish_only_feature_points").as_bool();
        // config_.use_imu_roll_pitch = this->get_parameter("laser_mapping_node.use_imu_roll_pitch").as_bool();
        config_.max_surface_features = this->get_parameter("laser_mapping_node.max_surface_features").as_int();
        config_.velocity_failure_threshold = this->get_parameter("laser_mapping_node.velocity_failure_threshold").as_double();
        config_.auto_voxel_size = this->get_parameter("laser_mapping_node.auto_voxel_size").as_bool();
        config_.forget_far_chunks = this->get_parameter("laser_mapping_node.forget_far_chunks").as_bool();
        config_.visual_confidence_factor = this->get_parameter("laser_mapping_node.visual_confidence_factor").as_double();
        config_.map_dir = this->get_parameter("map_dir").as_string(); 
        config_.localization_mode = this->get_parameter("laser_mapping_node.localization_mode").as_bool();
        config_.read_pose_file = this->get_parameter("laser_mapping_node.read_pose_file").as_bool();
        config_.use_imu_roll_pitch = USE_IMU_ROLL_PITCH;

        if(config_.read_pose_file)
        {   
            std::vector<utils::OdometryData> odometryResults;
            utils::readLocalizationPose(config_.map_dir, odometryResults);
            config_.init_x= odometryResults[0].x;
            config_.init_y= odometryResults[0].y;
            config_.init_z= odometryResults[0].z;
            config_.init_roll= odometryResults[0].roll;
            config_.init_pitch= odometryResults[0].pitch;
            config_.init_yaw= odometryResults[0].yaw;
        }
        else
        {  
            config_.init_x = get_parameter("laser_mapping_node.init_x").as_double(); 
            config_.init_y = get_parameter("laser_mapping_node.init_y").as_double(); 
            config_.init_z = get_parameter("laser_mapping_node.init_z").as_double(); 
            config_.init_roll = get_parameter("laser_mapping_node.init_roll").as_double();
            config_.init_pitch = get_parameter("laser_mapping_node.init_pitch").as_double();
            config_.init_yaw = get_parameter("laser_mapping_node.init_yaw").as_double(); 
        }

        return true;
    }
    
   

    void laserMapping::laserFeatureInfoHandler(const super_odometry_msgs::msg::LaserFeature::SharedPtr msgIn) {
       
        mBuf.lock();
        cornerLastBuf.push(msgIn->cloud_corner);
        surfLastBuf.push(msgIn->cloud_surface);
        realsenseBuf.push(msgIn->cloud_realsense);
        fullResBuf.push(msgIn->cloud_nodistortion);
        Eigen::Quaterniond imuprediction_tmp(msgIn->initial_quaternion_w, msgIn->initial_quaternion_x,
                                             msgIn->initial_quaternion_y, msgIn->initial_quaternion_z);

        IMUPredictionBuf.push(imuprediction_tmp);
        mBuf.unlock();
    }


void laserMapping::setInitialGuess()
{
  //Case1: First Frame Initialization 
  if(!initialization){
    initializeFirstFrame();
    return;
  }
  //Case2: Startup period -continue using IMU for stability 
  if(startupCount>0){
    initializeWithIMU();
    startupCount--;
    return; 
  }
  //Case3: Normal operation -select prediction source 
  selectPosePrediction();
}

void laserMapping::initializeFirstFrame(){

    //Get initial orientation from IMU prediction 
    if(sensorMeas.imuPrediction.w()!=0){   //Have IMU data
        //Extract roll and pitch, zero out yaw 
        tf2::Quaternion initial_orientation=utils::extractRollPitch(sensorMeas.imuPrediction);
        q_w_curr=Eigen::Quaterniond(initial_orientation.w(), initial_orientation.x(),
              initial_orientation.y(), initial_orientation.z());
        auto q_extrinsic=Eigen::Quaterniond(imu_laser_R);
        q_extrinsic.normalize();
        q_w_curr=q_extrinsic.inverse()*q_w_curr;
        
        
    }else{

        q_w_curr=Eigen::Quaterniond(1,0,0,0); //If no IMU data, use identity rotation 

    }

    //initialize position 
    q_wodom_pre=q_w_curr;
    T_w_lidar.rot=q_w_curr;
    T_w_lidar.pos=Eigen::Vector3d::Zero();

    //Overide with predefined pose if localization mode 
    if(slam.localization_mode){
        T_w_lidar.pos=Eigen::Vector3d(slam.init_x,slam.init_y,slam.init_z);
        tf2::Quaternion localization_pose;
        localization_pose.setRPY(slam.init_roll, slam.init_pitch,slam.init_yaw);
        T_w_lidar.rot=Eigen::Quaterniond(localization_pose.w(), localization_pose.x(),
                                        localization_pose.y(), localization_pose.z());
        slam.last_T_w_lidar=T_w_lidar;
    }

}

void laserMapping::initializeWithIMU(){
    if(sensorMeas.imuPrediction.w()!=0){  //Have IMU data
    //Use IMU Orientation directly during startup for seconds 
    tf2::Quaternion curr_imu(sensorMeas.imuPrediction.w(), sensorMeas.imuPrediction.x(),
                             sensorMeas.imuPrediction.y(), sensorMeas.imuPrediction.z());
    
    //Keep position from last frame 
    t_w_curr=last_T_w_lidar.pos;
    T_w_lidar.pos=t_w_curr;

    //Update rotation 
    q_w_curr=Eigen::Quaterniond(curr_imu.w(), curr_imu.x(), curr_imu.y(), curr_imu.z());
    T_w_lidar.rot=q_w_curr;


    }else
    {
      //No IMU data, use last rotation 
      q_w_curr=last_T_w_lidar.rot;
      t_w_curr=last_T_w_lidar.pos;
      T_w_lidar=last_T_w_lidar;

    } 
}

void laserMapping::selectPosePrediction(){

// Step1: Decide prediction source based on system state 
prediction_source=determinePredictionSource();

//Step2: Get prediction from selected source 
switch(prediction_source){
    case PredictionSource::LIO_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.lioPrediction;
    break;
    } 
   
    case PredictionSource::VIO_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.vioPrediction;
    break; 
    } 

    case PredictionSource::NEURAL_IMU_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.nioPrediction;
    break; 
    } 
    case PredictionSource::IMU_ORIENTATION:{
    Eigen::Quaterniond q_w_predict=q_w_curr*q_wodom_pre.inverse()*q_wodom_curr;
    q_w_predict.normalize();
    T_w_lidar.rot=q_w_predict;
    q_wodom_pre=q_wodom_curr;
    break;
    } 
   
    case PredictionSource::CONSTANT_VELOCITY:{  
    Transformd relative_pose=last_T_w_lidar.inverse()*T_w_lidar;
    T_w_lidar=T_w_lidar*relative_pose;
    break; 
    }
}

//Step4: Update current pose 
q_w_curr=T_w_lidar.rot;
t_w_curr=T_w_lidar.pos;

}

laserMapping::PredictionSource laserMapping::determinePredictionSource(){
// If system is degerenate, prefer VIO or learning imu odom

if(slam.isDegenerate){
    if(sensorMeas.vio_prediction_status){
        return PredictionSource::VIO_ODOM;
    }
    if(sensorMeas.nio_prediction_status){
        return PredictionSource::NEURAL_IMU_ODOM;
    }

}else{
    // If system is not degenerate, use IMU orientation 
    if(sensorMeas.lio_prediction_status){
        return PredictionSource::LIO_ODOM;
    }
    sensorMeas.imu_orientation_status=useIMUPrediction(sensorMeas.imuPrediction);
    if(sensorMeas.imu_orientation_status){
        return PredictionSource::IMU_ORIENTATION;
    }
   
}



// If no prediction source is available, use constant velocity
return PredictionSource::CONSTANT_VELOCITY;

}

    void laserMapping::publishTopic(){

        TicToc t_pub;
        std_msgs::msg::String prediction_source_msg;
        switch (prediction_source) {
            case PredictionSource::IMU_ORIENTATION :
                prediction_source_msg.data = "IMU Only Orientation Prediction";
                break;
            case PredictionSource::LIO_ODOM :
                prediction_source_msg.data = "Using Laser-Inertial Odometry (LIO)";
                break;
            case PredictionSource::VIO_ODOM :
                prediction_source_msg.data = "Using Visual-Inertial Odometry (VIO)";
                break;
            case PredictionSource::NEURAL_IMU_ODOM :
                prediction_source_msg.data = "Using Neural-Inertial Odometry (Neural-IMU)";
                break;
            case PredictionSource::CONSTANT_VELOCITY :
                prediction_source_msg.data = "Using Constant Velocity Prediction";
                break;
        }
        pubprediction_source->publish(prediction_source_msg);

        if (frameCount % 5 == 0 && config_.debug_view_enabled) {
            laserCloudSurround->clear();
            *laserCloudSurround = slam.localMap.get5x5LocalMap(slam.pos_in_localmap);
            sensor_msgs::msg::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
            laserCloudSurround3.header.stamp =
                    rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudSurround3.header.frame_id = WORLD_FRAME;
            pubLaserCloudSurround->publish(laserCloudSurround3);
        }

        if (frameCount % 20 == 0) {
            pcl::PointCloud<PointType> laserCloudMap;
            laserCloudMap = slam.localMap.getAllLocalMap();
            sensor_msgs::msg::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudMsg.header.frame_id = WORLD_FRAME;
            pubLaserCloudMap->publish(laserCloudMsg);
            
            if (slam.localization_mode) {
                priorCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
                pubLaserCloudPrior->publish(priorCloudMsg);
            }
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
            PointType const *const &pi = &laserCloudFullRes->points[i];
            if (pi->x* pi->x+ pi->y * pi->y + pi->z* pi->z < 0.01)
            {
                continue;
            }

            utils::pointAssociateToMap(&laserCloudFullRes->points[i],
                                &laserCloudFullRes->points[i],
                                q_w_curr,
                                t_w_curr);
        }

        pcl::PointCloud<pcl::PointXYZI> laserCloudFullResCvt, laserCloudFullResClean;
        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        pcl::fromROSMsg(laserCloudFullRes3, laserCloudFullResCvt);
        for (int i = 0; i < laserCloudFullResNum; i++) {
          PointType const *const &pi = &laserCloudFullResCvt.points[i];
          if (pi->x* pi->x+ pi->y * pi->y + pi->z* pi->z > 0.01)
          {
             laserCloudFullResClean.push_back(*pi);
          }
        }
        pcl::toROSMsg(laserCloudFullResClean, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
        laserCloudFullRes3.header.frame_id = WORLD_FRAME;
        pubLaserCloudFullRes->publish(laserCloudFullRes3);

        laserCloudFullResCvt.clear();
        laserCloudFullResClean.clear();
        laserCloudFullRes_rot->clear();
        laserCloudFullRes_rot->resize(laserCloudFullResNum);

        for (int i = 0; i < laserCloudFullResNum; i++) {
            laserCloudFullRes_rot->points[i].x = laserCloudFullRes->points[i].y;
            laserCloudFullRes_rot->points[i].y = laserCloudFullRes->points[i].z;
            laserCloudFullRes_rot->points[i].z = laserCloudFullRes->points[i].x;
            laserCloudFullRes_rot->points[i].intensity = laserCloudFullRes->points[i].intensity;
        }

        nav_msgs::msg::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = WORLD_FRAME;
        odomAftMapped.child_frame_id = SENSOR_FRAME;
        odomAftMapped.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);

        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();

        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();

        odomAftMapped.twist.twist.linear.x = vel_b.x();
        odomAftMapped.twist.twist.linear.y = vel_b.y();
        odomAftMapped.twist.twist.linear.z = vel_b.z();

        odomAftMapped.twist.twist.angular.x = ang_vel_b.x();
        odomAftMapped.twist.twist.angular.y = ang_vel_b.y();
        odomAftMapped.twist.twist.angular.z = ang_vel_b.z();

        nav_msgs::msg::Odometry laserOdomIncremental;

        if (initialization == false)
        {
            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = t_w_curr.x();
            laserOdomIncremental.pose.pose.position.y = t_w_curr.y();
            laserOdomIncremental.pose.pose.position.z = t_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.x = q_w_curr.x();
            laserOdomIncremental.pose.pose.orientation.y = q_w_curr.y();
            laserOdomIncremental.pose.pose.orientation.z = q_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.w = q_w_curr.w();
        }
        else
        {

            laser_incremental_T = T_w_lidar;
            laser_incremental_T.rot.normalized();

            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = laser_incremental_T.pos.x();
            laserOdomIncremental.pose.pose.position.y = laser_incremental_T.pos.y();
            laserOdomIncremental.pose.pose.position.z = laser_incremental_T.pos.z();
            laserOdomIncremental.pose.pose.orientation.x = laser_incremental_T.rot.x();
            laserOdomIncremental.pose.pose.orientation.y = laser_incremental_T.rot.y();
            laserOdomIncremental.pose.pose.orientation.z = laser_incremental_T.rot.z();
            laserOdomIncremental.pose.pose.orientation.w = laser_incremental_T.rot.w();
        }

        pubLaserOdometryIncremental->publish(laserOdomIncremental);


        if (slam.isDegenerate) {
            odomAftMapped.pose.covariance[0] = 1;
        } else {
            odomAftMapped.pose.covariance[0] = 0;
        }

        rclcpp::Time pub_time = rclcpp::Clock{RCL_ROS_TIME}.now(); //PARV_TODO - find how to syncrynoise this with rosbag time
        pubOdomAftMapped->publish(odomAftMapped);

        geometry_msgs::msg::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = WORLD_FRAME;
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath->publish(laserAfterMappedPath);


        slam.stats.header = odomAftMapped.header;
        if (timeLatestImuOdometry.seconds() < 1.0)
        {
            timeLatestImuOdometry = pub_time;
        }
        rclcpp::Duration latency = timeLatestImuOdometry - pub_time;  
        slam.stats.latency = latency.seconds() * 1000;
        slam.stats.n_iterations = slam.stats.iterations.size();
        // Avoid breaking rqt_multiplot
        while (slam.stats.iterations.size() < 4)
        {
            slam.stats.iterations.push_back(super_odometry_msgs::msg::IterationStats());
        }

        pubOptimizationStats->publish(slam.stats);
        slam.stats.iterations.clear();
    }


    void laserMapping::adjustVoxelSize(){

        // Calculate cloud statistics
        bool increase_blind_radius = false;
        if(config_.auto_voxel_size)
        {
            Eigen::Vector3f average(0,0,0);
            int count_far_points = 0;
            for (auto &point : *laserCloudSurfLast)
            {
                average(0) += fabs(point.x);
                average(1) += fabs(point.y);
                average(2) += fabs(point.z);
                if(point.x*point.x + point.y*point.y + point.z*point.z>9){
                    count_far_points++;
                }
            }
            if (count_far_points > 3000)
            {
                increase_blind_radius = true;
            }

            average /= laserCloudSurfLast->points.size();
            slam.stats.average_distance = average(0)*average(1)*average(2);
            if (slam.stats.average_distance < 25)
            {
                config_.lineRes = 0.1;
                config_.planeRes = 0.2;
            }
            else if (slam.stats.average_distance > 65)
            {
                config_.lineRes = 0.4;
                config_.planeRes = 0.8;
            }
            downSizeFilterSurf.setLeafSize(config_.planeRes , config_.planeRes , config_.planeRes );
            downSizeFilterCorner.setLeafSize(config_.lineRes , config_.lineRes , config_.lineRes );
        }

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);


        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
      

        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;

    }

    
    bool  laserMapping::checkDataAvailable() const{
        return !cornerLastBuf.empty() && !surfLastBuf.empty() 
               && !fullResBuf.empty() && !IMUPredictionBuf.empty();       
        //Note: in pure laser odometry, IMU Prediction will be identy. 
    }

    laserMapping::SensorData laserMapping::extractSensorData(){
        
        SensorData data;
        //1. Extract timestamp
        data.timestamp=secs(&fullResBuf.front());
        timeLaserOdometry=data.timestamp;

        //2. Extract point cloud data 
        pcl::fromROSMsg(cornerLastBuf.front(), *laserCloudCornerLast);
        cornerLastBuf.pop();
        pcl::fromROSMsg(surfLastBuf.front(), *laserCloudSurfLast);
        surfLastBuf.pop();
        pcl::fromROSMsg(fullResBuf.front(), *laserCloudFullRes);
        fullResBuf.pop();

        //3. Extract IMU prediction 
        data.imuPrediction=IMUPredictionBuf.front();
        data.imuPrediction.normalize();
        IMUPredictionBuf.pop();

        //4 set status for prediction source (TODO: didn't release code other prediction source yet) 
        data.vio_prediction_status=false;
        data.lio_prediction_status=false;
        data.nio_prediction_status=false;
        data.imu_orientation_status=false;

        return data;
    }

    void laserMapping::clearSensorData(){
        auto clearBuffer=[](auto&buffer){
            while(!buffer.empty()){
                buffer.pop();
            }
        };
        clearBuffer(cornerLastBuf);
        clearBuffer(surfLastBuf);
        clearBuffer(fullResBuf);
        clearBuffer(IMUPredictionBuf);
    }


    /**
     * [功能描述]：执行SLAM优化（点云配准和位姿估计）
     * 该函数完成以下任务：
     * 1. 根据配置决定是否使用IMU的横滚角(roll)和俯仰角(pitch)作为约束
     * 2. 调用SLAM定位函数进行点云配准和位姿优化
     * 
     * 说明：
     * - 使用IMU姿态可以提高退化场景下的鲁棒性（如走廊、隧道等）
     * - 某些传感器（如Livox mid360）可能不需要使用roll/pitch约束
     * 
     * @return 无返回值，优化结果通过slam对象的成员变量返回
     */
    void laserMapping::performSLAMOptimization(){
        //! ========== 第一步：配置IMU姿态约束 ==========
        tf2::Quaternion imu_roll_pitch;
        
        // 检查是否启用IMU的横滚角和俯仰角作为优化约束
        if(config_.use_imu_roll_pitch){  
            // TODO: Livox mid360等某些传感器不使用roll/pitch角约束
            
            // 启用IMU姿态约束标志
            slam.OptSet.use_imu_roll_pitch=true;
            
            // 从IMU预测的四元数中提取横滚角(roll)和俯仰角(pitch)
            // 注意：这里只提取roll和pitch，yaw角由激光里程计优化得到
            imu_roll_pitch=utils::extractRollPitch(sensorMeas.imuPrediction);
            
            // 将提取的roll/pitch角设置到SLAM优化器中作为约束
            slam.OptSet.imu_roll_pitch=imu_roll_pitch;
        }else{
            // 不使用IMU姿态约束
            slam.OptSet.use_imu_roll_pitch=false;
            
            // 设置为单位四元数(0,0,0,1)，表示无旋转约束
            slam.OptSet.imu_roll_pitch=tf2::Quaternion(0,0,0,1);
        }

        //! ========== 第二步：执行SLAM定位和优化 ==========
        /**
         * SLAM定位函数参数说明：
         * @param initialization - 是否为初始化阶段（影响优化策略）
         * @param prediction_source - 预测源类型（IMU/VIO/LIO等）
         * @param T_w_lidar - 输入/输出参数：世界坐标系到激光雷达的变换矩阵（位姿初值和优化结果）
         * @param laserCloudCornerStack - 当前帧的角点（线特征）点云
         * @param laserCloudSurfStack - 当前帧的平面点云
         * @param timeLaserOdometry - 当前帧的时间戳
         */
        slam.Localization(initialization, static_cast<LidarSLAM::PredictionSource>(prediction_source), T_w_lidar,
                 laserCloudCornerStack, laserCloudSurfStack, timeLaserOdometry);
    }


    bool laserMapping::useIMUPrediction(const Eigen::Quaterniond& imuPrediction){
        if (imuPrediction.w()!=0)
        {
            q_wodom_curr=imuPrediction;
            q_wodom_curr.normalize();
            return true;
        }
        else
        {
            return false;
        }
    }

    void laserMapping::updatePoseAndPublish(){

        //1. Update pose 
        q_w_curr=slam.T_w_lidar.rot;
        t_w_curr=slam.T_w_lidar.pos;
        T_w_lidar.rot=slam.T_w_lidar.rot;
        T_w_lidar.pos=slam.T_w_lidar.pos;
        startupCount=slam.startupCount;
        frameCount++;
        slam.frame_count=frameCount;
        slam.laser_imu_sync=laser_imu_sync;
        initialization = true;

        // Calculate linear and angular velocity
        double dt = timeLaserOdometry - timeLaserOdometryPrev;

        if (dt > 1e-6) {  
            Eigen::Vector3d vel_w = (t_w_curr - last_T_w_lidar.pos) / dt;
            vel_b = q_w_curr.inverse() * vel_w;     

            Eigen::Quaterniond dq = q_w_curr * last_T_w_lidar.rot.inverse();
            Eigen::AngleAxisd angle_axis(dq);
            Eigen::Vector3d ang_vel_w = angle_axis.axis() * angle_axis.angle() / dt;
            ang_vel_b = q_w_curr.inverse() * ang_vel_w;
        } else {
            vel_b = Eigen::Vector3d::Zero();
            ang_vel_b = Eigen::Vector3d::Zero();
        }

        //2. Publish results 
        publishTopic();

        //3. Store current pose and time for next iteration
        last_T_w_lidar = slam.T_w_lidar;
        timeLaserOdometryPrev = timeLaserOdometry;
    }

    /**
     * [功能描述]：激光建图的主处理循环函数
     * 该函数持续运行，执行以下核心任务：
     * 1. 检查传感器数据是否可用
     * 2. 从缓冲区中提取传感器数据（包括点云、IMU等）
     * 3. 设置位姿初始猜测值（基于IMU预积分或运动模型）
     * 4. 根据运动速度自适应调整体素滤波器大小
     * 5. 执行SLAM优化（点云配准和位姿优化）
     * 6. 更新系统状态并发布结果
     * @return 无返回值
     */
    void laserMapping::process() {

        // 主循环：持续处理传感器数据直到ROS节点关闭
        while (rclcpp::ok()) {
            //! ========== 第一步：检查数据可用性 ==========
            // 检查是否有新的传感器数据到达
            if(!checkDataAvailable()){
                // 如果没有数据，休眠2ms后继续检查，避免空转消耗CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            
            // 使用try-catch捕获处理过程中的异常，确保系统稳定性
            try{
                //! ========== 第二步：开始帧处理计时 ==========
                // 创建计时器，用于统计单帧处理时间
                utils::ScopedTimer timer("Frame Processing");
                
                //! ========== 第三步：提取传感器数据（线程安全） ==========
                // 加锁保护共享数据缓冲区，防止多线程竞争
                mBuf.lock(); 
                // 从缓冲区提取当前帧的传感器测量数据（点云、IMU、时间戳等）
                sensorMeas=extractSensorData();
                // 清空已提取的传感器数据，释放缓冲区空间
                clearSensorData();
                // 解锁，允许其他线程访问缓冲区
                mBuf.unlock();
                
                //! ========== 第四步：设置位姿初始猜测 ==========
                // 基于IMU预积分、运动模型或上一帧位姿，设置当前帧位姿的初始估计值
                // 好的初始猜测可以加速优化收敛并提高配准成功率
                setInitialGuess();
                
                //! ========== 第五步：自适应调整体素大小 ==========
                // 根据机器人运动速度动态调整点云降采样的体素大小
                // 高速运动时使用较大体素以提高鲁棒性，低速时使用较小体素以提高精度
                adjustVoxelSize();
                
                //! ========== 第六步：执行SLAM优化 ==========
                // 执行核心SLAM算法：
                // - 提取局部地图
                // - 点云配准（ICP/点到面优化）
                // - 位姿图优化
                // - 退化检测和处理
                performSLAMOptimization();
                
                //! ========== 第七步：更新位姿并发布结果 ==========
                // 更新系统状态（位姿、速度、地图等）
                // 发布里程计、点云、路径等ROS话题
                updatePoseAndPublish();
               
                //! ========== 第八步：更新统计和调试信息（已注释） ==========
                //updateStatsAndDebugInfo();

            }catch(const std::exception&e){
                // 捕获并记录处理过程中的异常，避免程序崩溃
                RCLCPP_ERROR(this->get_logger(), "Error in frame processing: %s", e.what());
            }
        }

    }


#pragma clang diagnostic pop

} // namespace super_odometry
