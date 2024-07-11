#ifndef ODOM_NODE_HPP_
#define ODOM_NODE_HPP_

#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "image_transport/image_transport.hpp"

class OdomNode : public rclcpp::Node {
public:
    OdomNode();

    // Getter methods for all parameters
    const std::string& getSeqDir() const;
    const std::string& getAlgorithm() const;
    const std::string& getDataset() const;
    const std::string& getKeypointExtraction() const;
    const std::string& getOdomFrame() const;
    const std::string& getChildFrame() const;
    int getEndIdx() const;
    bool getDoSlam() const;
    bool getVizExtraction() const;
    bool getVizMatching() const;
    bool getStopEachFrame() const;
    double getFrameRate() const;

    // ORORA params
    bool getEstimatingScale() const;
    int getNumMaxIter() const;
    double getNoiseBound() const;
    double getNoiseBoundRadial() const;
    double getNoiseBoundTangential() const;
    double getNoiseBoundCoeff() const;
    double getGncFactor() const;
    double getRotCostDiffThr() const;

    // DOPPLER params
    bool getCheckStopMotion() const;
    int getNumFeatThrForStopMotion() const;
    double getBeta() const;

    // Feature Matching params
    double getVoxelSize() const;
    bool getUseVoxelization() const;
    bool getUseDopplerCompensation() const;
    bool getUseDeskewing() const;
    const std::string& getDeskewingTarget() const;
    
    // Publisher Functions
    void publishGT(const nav_msgs::msg::Odometry& message);
    void publishOdom(const nav_msgs::msg::Odometry& message);
    void publishLaserCloudLocal(const sensor_msgs::msg::PointCloud2& message);
    void publishLaserCloudGlobal(const sensor_msgs::msg::PointCloud2& message);
    void publishPrevFeat(const sensor_msgs::msg::PointCloud2& message);
    void publishPrevCompensated(const sensor_msgs::msg::PointCloud2& message);
    void publishCurrFeat(const sensor_msgs::msg::PointCloud2& message);
    void publishCurrCompensated(const sensor_msgs::msg::PointCloud2& message);
    void publishImg(const sensor_msgs::msg::Image& message);


private:
    // all publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr PubGT;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr PubOdom;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubLaserCloudLocal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubLaserCloudGlobal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubPrevFeat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubPrevCompensated;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubCurrFeat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PubCurrCompensated;
    image_transport::Publisher PubImg;
    std::shared_ptr<image_transport::ImageTransport> it_;

    // define all Parameters
    bool do_slam, viz_extraction, viz_matching, stop_each_frame;
    std::string seq_dir, algorithm, dataset;
    std::string keypoint_extraction; // "cen2018", "cen2019", "orb"
    std::string odom_frame;          // "odom";
    std::string child_frame;         // "radar_base";
    double frame_rate;               // 4 Hz
    int end_idx;                     // -1, because radar_files.size() - 1;

    // ORORA Params
    bool estimating_scale;
    int num_max_iter;
    double noise_bound, noise_bound_radial, noise_bound_tangential;
    double noise_bound_coeff, gnc_factor, rot_cost_diff_thr, voxel_size;

    // DOPPLER params
    bool check_stop_motion;             // Implementation details: check stop motion
    int num_feat_thr_for_stop_motion;
    double beta;

    // Feature Matching Params
    bool use_voxelization, use_doppler_compensation, use_deskewing;
    std::string deskewing_target;

    // Helper method to setup parameters
    void setupParameters();
};

#endif /* ODOM_NODE_HPP_ */

