#include <math.h>
//#include <cmath> C++ konform
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;
using std::cout;
using std::endl;

double keyframeMeterGap;
double movementAccumulation = 1000000.0; // large value means must add the first given frame.
bool isNowKeyFrame = false; 

std::queue<std::shared_ptr<nav_msgs::msg::Odometry const>> odometryBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2 const>> fullResBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::NavSatFix const>> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
std::shared_ptr<const sensor_msgs::msg::NavSatFix> currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false; 
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;



class LaserPGONode : public rclcpp::Node {
    public:
    	LaserPGONode() : Node("laserPGO") {
	    this->declare_parameter<double>("keyframe_meter_gap", 2.0); // pose assignment every k frames
	    this->declare_parameter<double>("sc_dist_thres", 0.2); // pose assignment every k frames

	    this->get_parameter("keyframe_meter_gap", keyframeMeterGap);
	    this->get_parameter("sc_dist_thres", scDistThres);

	    ISAM2Params parameters;
	    parameters.relinearizeThreshold = 0.01;
	    parameters.relinearizeSkip = 1;
	    isam = new ISAM2(parameters);
	    LaserPGONode::initNoises();

	    scManager.setSCdistThres(scDistThres);

	    float filter_size = 0.4; 
	    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
	    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

	    float map_vis_size = 0.2;
	    downSizeFilterMapPGO.setLeafSize(map_vis_size, map_vis_size, map_vis_size);
        /*
	    subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
	    	"/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

	    subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
	    	"/aft_mapped_to_init", 100, laserOdometryHandler);

	    subGPS = this->create_subscription<sensor_msgs::msg::NavSatFix>(
	    	"/gps/fix", 100, gpsHandler);
        */
        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_registered_local", 100, std::bind(&LaserPGONode::laserCloudFullResHandler, this, std::placeholders::_1));

        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", 100, std::bind(&LaserPGONode::laserOdometryHandler, this, std::placeholders::_1));

        subGPS = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 100, std::bind(&LaserPGONode::gpsHandler, this, std::placeholders::_1));


	    pubOdomAftPGO = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", rclcpp::QoS(100));
	    pubOdomRepubVerifier = this->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", rclcpp::QoS(100));
	    pubPathAftPGO = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", rclcpp::QoS(100));
	    pubMapAftPGO = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", rclcpp::QoS(100));

	    pubLoopScanLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", rclcpp::QoS(100));
	    pubLoopSubmapLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", rclcpp::QoS(100));

	    posegraph_slam = std::thread(&LaserPGONode::process_pg, this); // pose graph construction
	    lc_detection = std::thread(&LaserPGONode::process_lcd, this); // loop closure detection 
	    icp_calculation = std::thread(&LaserPGONode::process_icp, this); // loop constraint calculation via icp 
	    viz_map = std::thread(&LaserPGONode::process_viz_map, this); // visualization - map (low frequency because it is heavy)
	    viz_path = std::thread(&LaserPGONode::process_viz_path, this); // visualization - path (high frequency)
        }

    private:
        // Subscriptions
    	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;

        // Publications
    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
    	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
	
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;

        // Threads
    	std::thread posegraph_slam;
    	std::thread lc_detection;
    	std::thread icp_calculation;
    	std::thread viz_map;
    	std::thread viz_path;

        // Methode declaration
        void laserOdometryHandler(const std::shared_ptr<const nav_msgs::msg::Odometry> &_laserOdometry);
        void laserCloudFullResHandler(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &_laserCloudFullRes);
        void gpsHandler(const std::shared_ptr<const sensor_msgs::msg::NavSatFix> &_gps);
        void initNoises();
        Pose6D getOdom(const std::shared_ptr<const nav_msgs::msg::Odometry> _odom);
        double transDiff(const Pose6D& pose1, const Pose6D& pose2);
        gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& pose);
        pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf);
        void pubPath();
        void updatePoses();
        void runISAM2opt();
        pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn);
        void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx);
        std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx);
        void process_pg();
        void performSCLoopClosure();
        void process_lcd();
        void process_icp();
        void process_viz_path();
        void pubMap();
        void process_viz_map();
        
};



void LaserPGONode::laserOdometryHandler(const std::shared_ptr<const nav_msgs::msg::Odometry> &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void LaserPGONode::laserCloudFullResHandler(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void LaserPGONode::gpsHandler(const std::shared_ptr<const sensor_msgs::msg::NavSatFix> &_gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void LaserPGONode::initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D LaserPGONode::getOdom(const std::shared_ptr<const nav_msgs::msg::Odometry> _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // cout << "curr given odom is " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << endl;
    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

double LaserPGONode::transDiff(const Pose6D& _p1, const Pose6D& _p2)
{
    return sqrt( (_p1.x - _p2.x)*(_p1.x - _p2.x) + (_p1.y - _p2.y)*(_p1.y - _p2.y) + (_p1.z - _p2.z)*(_p1.z - _p2.z) );
} // transDiff

gtsam::Pose3 LaserPGONode::Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

pcl::PointCloud<PointType>::Ptr LaserPGONode::local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void LaserPGONode::pubPath( void )
{
    // pub odom and path 
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "odom";
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "odom";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        // ROS1
        //odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);

        // ROS2
        tf2::Quaternion quat;
        quat.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(quat);

        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "odom";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO->publish(odomAftPGO); // last pose 
    pubPathAftPGO->publish(pathAftPGO); // poses 

    // HT comment: this TF leads to a conflict with that in ORORA
//    static tf2_ros::TransformBroadcaster br;
//    tf2::Transform transform;
//    tf2::Quaternion q;
//    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
//    q.setW(odomAftPGO.pose.pose.orientation.w);
//    q.setX(odomAftPGO.pose.pose.orientation.x);
//    q.setY(odomAftPGO.pose.pose.orientation.y);
//    q.setZ(odomAftPGO.pose.pose.orientation.z);
//    transform.setRotation(q);
//    br.sendTransform(tf2::StampedTransform(transform, odomAftPGO.header.stamp, "odom", "aft_pgo"));
} // pubPath


void LaserPGONode::updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();
    mtxRecentPose.unlock();
} // updatePoses

void LaserPGONode::runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    LaserPGONode::updatePoses();
}

pcl::PointCloud<PointType>::Ptr LaserPGONode::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void LaserPGONode::loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= static_cast<int>(keyframeLaserClouds.size()) )
            continue;

        mKF.lock(); 
        *nearKeyframes += *LaserPGONode::local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
        mKF.unlock(); 
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


std::optional<gtsam::Pose3> LaserPGONode::doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    LaserPGONode::loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    LaserPGONode::loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // loop verification 
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "odom";
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "odom";
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative


void LaserPGONode::process_pg()
{
    while(1)
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && rclcpp::Time(odometryBuf.front()->header.stamp).seconds() < rclcpp::Time(fullResBuf.front()->header.stamp).seconds()) {
                odometryBuf.pop();
            }
            if (odometryBuf.empty()) {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = LaserPGONode::getOdom(odometryBuf.front());
            odometryBuf.pop();

            // find nearest gps 
            double eps = 0.1; // find a gps topioc arrived within eps second 
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true; 
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock(); 

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            double delta_translation = LaserPGONode::transDiff(odom_pose_prev, odom_pose_curr);
            movementAccumulation += delta_translation;

            if( movementAccumulation > keyframeMeterGap ) {
                isNowKeyFrame = true;
                movementAccumulation = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame 
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                } 
            }

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = LaserPGONode::Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    LaserPGONode::runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                const int prev_node_idx = keyframePoses.size() - 2; 
                const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
                gtsam::Pose3 poseFrom = LaserPGONode::Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = LaserPGONode::Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // gps factor 
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set) 
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);                
                    LaserPGONode::runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 5 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg


void LaserPGONode::performSCLoopClosure(void)
{
    if( keyframePoses.size() < static_cast<size_t>(scManager.NUM_EXCLUDE_RECENT)) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure


void LaserPGONode::process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        LaserPGONode::performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd



void LaserPGONode::process_icp(void)
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(this->get_logger(), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = LaserPGONode::doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                LaserPGONode::runISAM2opt();
                mtxPosegraph.unlock();
            } 
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp


void LaserPGONode::process_viz_path(void)
{
    float hz = 5.0; 
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(keyframePosesUpdated.size() > 1) {
            LaserPGONode::pubPath();
        }
    }
}


void LaserPGONode::pubMap(void)
{
    int SKIP_FRAMES = 2;
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock(); 
    for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *LaserPGONode::local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "odom";
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}


void LaserPGONode::process_viz_map(void)
{
    float vizmapFrequency = 0.1;
    // float vizmapFrequency = 1.0;
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(keyframeLaserClouds.size() > 1) {
            LaserPGONode::pubMap();
        }
    }
} // pointcloud_viz


/*
class LaserPGONode : public rclcpp::Node {
    public:
    	LaserPGONode() : Node("laserPGO") {
	    this->declare_parameter<double>("keyframe_meter_gap", 2.0); // pose assignment every k frames
	    this->declare_parameter<double>("sc_dist_thres", 0.2); // pose assignment every k frames

	    this->get_parameter("keyframe_meter_gap", keyframeMeterGap);
	    this->get_parameter("sc_dist_thres", scDistThres);

	    ISAM2Params parameters;
	    parameters.relinearizeThreshold = 0.01;
	    parameters.relinearizeSkip = 1;
	    isam = new ISAM2(parameters);
	    initNoises();

	    scManager.setSCdistThres(scDistThres);

	    float filter_size = 0.4; 
	    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
	    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

	    float map_vis_size = 0.2;
	    downSizeFilterMapPGO.setLeafSize(map_vis_size, map_vis_size, map_vis_size);

	    subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
	    	"/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

	    subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
	    	"/aft_mapped_to_init", 100, laserOdometryHandler);

	    subGPS = this->create_subscription<sensor_msgs::msg::NavSatFix>(
	    	"/gps/fix", 100, gpsHandler);

	    pubOdomAftPGO = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", rclcpp::QoS(100));
	    pubOdomRepubVerifier = this->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", rclcpp::QoS(100));
	    pubPathAftPGO = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", rclcpp::QoS(100));
	    pubMapAftPGO = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", rclcpp::QoS(100));

	    pubLoopScanLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", rclcpp::QoS(100));
	    pubLoopSubmapLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", rclcpp::QoS(100));

	    posegraph_slam = std::thread(&process_pg, this); // pose graph construction
	    lc_detection = std::thread(&process_lcd, this); // loop closure detection 
	    icp_calculation = std::thread(&LaserPGONode::process_icp, this); // loop constraint calculation via icp 
	    viz_map = std::thread(&process_viz_map, this); // visualization - map (low frequency because it is heavy)
	    viz_path = std::thread(&process_viz_path, this); // visualization - path (high frequency)
        }

    private:
    	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
	
    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
    	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
	
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;
	
    	std::thread posegraph_slam;
    	std::thread lc_detection;
    	std::thread icp_calculation;
    	std::thread viz_map;
    	std::thread viz_path;

        void process_icp(void);
        void pubPath(void);
        std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx );
        void pubMap(void);
};
*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserPGONode>());
    // rclcpp::shutdown();
    return 0;
}
