#include "OdomNode.hpp"

OdomNode::OdomNode()
    : Node("orora") {
    // create Publisher
    PubGT = this->create_publisher<nav_msgs::msg::Odometry>("/orora/gt", rclcpp::QoS(100));
    PubOdom = this->create_publisher<nav_msgs::msg::Odometry>("/orora/odom", rclcpp::QoS(100));
    PubLaserCloudLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/cloud_local", rclcpp::QoS(100));
    PubLaserCloudGlobal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/cloud_global", rclcpp::QoS(100));
    PubPrevFeat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/prev/feat", rclcpp::QoS(100));
    PubPrevCompensated = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/prev/compensated", rclcpp::QoS(100));
    PubCurrFeat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/curr/feat", rclcpp::QoS(100));
    PubCurrCompensated = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orora/curr/compensated", rclcpp::QoS(100));
    // initialize ImageTransport after Node is created
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    PubImg = it_->advertise("orora/matched_img", 100);

    setupParameters();

    std::cout << "\033[1;32mTarget dataset: " << dataset << " \033[0m" << std::endl;
    std::cout << "\033[1;32mTarget algorithm: " << algorithm << " \033[0m" << std::endl;
}

void OdomNode::setupParameters() {
    // declare and get params
    this->declare_parameter<std::string>("seq_dir", "");
    this->declare_parameter<std::string>("algorithm", "");
    this->declare_parameter<std::string>("dataset", "");
    this->declare_parameter<std::string>("keypoint_extraction", "cen2019");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("child_frame", "radar_base");
    this->declare_parameter<int>("end_idx", -1);
    this->declare_parameter<bool>("do_slam", false);
    this->declare_parameter<bool>("viz_extraction", false);
    this->declare_parameter<bool>("viz_matching", false);
    this->declare_parameter<bool>("stop_each_frame", false);
    this->declare_parameter<double>("frame_rate", 4.0);
    this->get_parameter("seq_dir", seq_dir);
    this->get_parameter("algorithm", algorithm);
    this->get_parameter("dataset", dataset);
    this->get_parameter("keypoint_extraction", keypoint_extraction);
    this->get_parameter("odom_frame", odom_frame);
    this->get_parameter("child_frame", child_frame);
    this->get_parameter("end_idx", end_idx);
    this->get_parameter("do_slam", do_slam);
    this->get_parameter("viz_extraction", viz_extraction);
    this->get_parameter("viz_matching", viz_matching);
    this->get_parameter("stop_each_frame", stop_each_frame);
    this->get_parameter("frame_rate", frame_rate);

    // ORORA params
    this->declare_parameter<bool>("/ORORA/estimating_scale", false);
    this->declare_parameter<double>("/ORORA/noise_bound", 0.5);
    this->declare_parameter<double>("/ORORA/noise_bound_radial", 0.3536);
    this->declare_parameter<double>("/ORORA/noise_bound_tangential", 9.0);
    this->declare_parameter<double>("/ORORA/noise_bound_coeff", 1.0);
    this->declare_parameter<double>("/ORORA/rotation/gnc_factor", 1.39);
    this->declare_parameter<double>("/ORORA/rotation/rot_cost_diff_thr", 0.0001);
    this->declare_parameter<int>("/ORORA/rotation/num_max_iter", 50);
    this->get_parameter("/ORORA/estimating_scale", estimating_scale);
    this->get_parameter("/ORORA/noise_bound", noise_bound);
    this->get_parameter("/ORORA/noise_bound_radial", noise_bound_radial);
    this->get_parameter("/ORORA/noise_bound_tangential", noise_bound_tangential);
    this->get_parameter("/ORORA/noise_bound_coeff", noise_bound_coeff);
    this->get_parameter("/ORORA/rotation/gnc_factor", gnc_factor);
    this->get_parameter("/ORORA/rotation/rot_cost_diff_thr", rot_cost_diff_thr);
    this->get_parameter("/ORORA/rotation/num_max_iter", num_max_iter);

    // DOPPLER params
    this->declare_parameter<bool>("/ORORA/stop_motion/check_stop_motion", true);
    this->declare_parameter<int>("/ORORA/stop_motion/num_feat_thr_for_stop_motion", 600);
    this->get_parameter("/ORORA/stop_motion/check_stop_motion", check_stop_motion);
    this->get_parameter("/ORORA/stop_motion/num_feat_thr_for_stop_motion", num_feat_thr_for_stop_motion);

    // Feature matching params
    this->declare_parameter<double>("/ORORA/voxel_size", 0.5);
    this->declare_parameter<bool>("/ORORA/use_voxelization", false);
    this->declare_parameter<bool>("/ORORA/use_doppler_compensation", false);
    this->declare_parameter<bool>("/ORORA/use_deskewing", false);
    this->declare_parameter<std::string>("/ORORA/deskewing_target", "rot");
    this->get_parameter("/ORORA/voxel_size", voxel_size);
    this->get_parameter("/ORORA/use_voxelization", use_voxelization);
    this->get_parameter("/ORORA/use_doppler_compensation", use_doppler_compensation);
    this->get_parameter("/ORORA/use_deskewing", use_deskewing);
    this->get_parameter("/ORORA/deskewing_target", deskewing_target);
}

// getter for parameters
const std::string& OdomNode::getSeqDir() const { return seq_dir; }
const std::string& OdomNode::getAlgorithm() const { return algorithm; }
const std::string& OdomNode::getDataset() const { return dataset; }
const std::string& OdomNode::getKeypointExtraction() const { return keypoint_extraction; }
const std::string& OdomNode::getOdomFrame() const { return odom_frame; }
const std::string& OdomNode::getChildFrame() const { return child_frame; }
int OdomNode::getEndIdx() const { return end_idx; }
bool OdomNode::getDoSlam() const { return do_slam; }
bool OdomNode::getVizExtraction() const { return viz_extraction; }
bool OdomNode::getVizMatching() const { return viz_matching; }
bool OdomNode::getStopEachFrame() const { return stop_each_frame; }
double OdomNode::getFrameRate() const { return frame_rate; }

bool OdomNode::getEstimatingScale() const { return estimating_scale; }
int OdomNode::getNumMaxIter() const { return num_max_iter; }
double OdomNode::getNoiseBound() const { return noise_bound; }
double OdomNode::getNoiseBoundRadial() const { return noise_bound_radial; }
double OdomNode::getNoiseBoundTangential() const { return noise_bound_tangential; }
double OdomNode::getNoiseBoundCoeff() const { return noise_bound_coeff; }
double OdomNode::getGncFactor() const { return gnc_factor; }
double OdomNode::getRotCostDiffThr() const { return rot_cost_diff_thr; }

bool OdomNode::getCheckStopMotion() const { return check_stop_motion; }
int OdomNode::getNumFeatThrForStopMotion() const { return num_feat_thr_for_stop_motion; }
double OdomNode::getBeta() const { return beta; }

double OdomNode::getVoxelSize() const { return voxel_size; }
bool OdomNode::getUseVoxelization() const { return use_voxelization; }
bool OdomNode::getUseDopplerCompensation() const { return use_doppler_compensation; }
bool OdomNode::getUseDeskewing() const { return use_deskewing; }
const std::string& OdomNode::getDeskewingTarget() const { return deskewing_target; }


// publish functions
void OdomNode::publishGT(const nav_msgs::msg::Odometry& message) {
    PubGT->publish(message); }

void OdomNode::publishOdom(const nav_msgs::msg::Odometry& message) {
    PubOdom->publish(message); }

void OdomNode::publishLaserCloudLocal(const sensor_msgs::msg::PointCloud2& message) {
    PubLaserCloudLocal->publish(message); }

void OdomNode::publishLaserCloudGlobal(const sensor_msgs::msg::PointCloud2& message) {
    PubLaserCloudGlobal->publish(message); }

void OdomNode::publishPrevFeat(const sensor_msgs::msg::PointCloud2& message) {
    PubPrevFeat->publish(message); }

void OdomNode::publishPrevCompensated(const sensor_msgs::msg::PointCloud2& message) {
    PubPrevCompensated->publish(message); }

void OdomNode::publishCurrFeat(const sensor_msgs::msg::PointCloud2& message) {
    PubCurrFeat->publish(message); }

void OdomNode::publishCurrCompensated(const sensor_msgs::msg::PointCloud2& message) {
    PubCurrCompensated->publish(message); }

void OdomNode::publishImg(const sensor_msgs::msg::Image& message) {
    PubImg.publish(message); }


