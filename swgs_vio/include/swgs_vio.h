#ifndef SWGS_VIO_H
#define SWGS_VIO_H

#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

namespace swgs_vio {

// 定义滑动窗口的大小
const int SLIDING_WINDOW_SIZE = 10;

// 状态结构体，表示滑动窗口中的每一帧状态
struct State {
    double timestamp; // 时间戳
    Eigen::Vector3d position; // 位置
    Eigen::Quaterniond orientation; // 姿态（四元数）
    Eigen::Vector3d velocity; // 速度
    Eigen::Matrix<double, 15, 15> covariance; // 状态协方差矩阵

    State() : timestamp(0.0), position(Eigen::Vector3d::Zero()),
              orientation(Eigen::Quaterniond::Identity()),
              velocity(Eigen::Vector3d::Zero()),
              covariance(Eigen::Matrix<double, 15, 15>::Identity()) {}
};

// 地图结构，用3D Gaussian表示

struct MapGaussian {
    Eigen::Vector3d position;     // 3D 位置
    Eigen::Matrix3d covariance;   // 协方差矩阵
    Eigen::Vector3d color;        // 颜色（RGB）
    float opacity;                // 不透明度
    Eigen::Vector3d velocity;     // 速度（可选）
    double timestamp;             // 时间戳

    // 构造函数
    MapGaussian(const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov,
                const Eigen::Vector3d& col, float opac,
                const Eigen::Vector3d& vel = Eigen::Vector3d::Zero(),
                double time = 0.0)
        : position(pos), covariance(cov), color(col), opacity(opac), velocity(vel), timestamp(time) {}
};

// 滑动窗口图优化类
class SlidingWindowGSVIO {
public:
    SlidingWindowGSVIO();
    ~SlidingWindowGSVIO();

    // 初始化滑动窗口
    void initialize(const Eigen::Vector3d& initial_position,
                    const Eigen::Quaterniond& initial_orientation,
                    const Eigen::Vector3d& initial_velocity);

    // 处理IMU数据
    void processIMUData(const sensor_msgs::Imu::ConstPtr& imu_msg);

    // 处理图像数据
    void processImageData(const sensor_msgs::Image::ConstPtr& image_msg);

    // 更新滑动窗口
    void updateSlidingWindow();

    // 获取当前状态
    State getCurrentState() const;

    // 获取滑动窗口中的所有状态
    std::deque<State> getSlidingWindowStates() const;

    // 获取地图点（用于 3D Gaussian Splatting）
    std::vector<MapPoint> getMapPoints() const;

private:
    // 初始化标志
    bool is_initialized_;

    // 滑动窗口状态队列
    std::deque<State> sliding_window_;

    // 当前状态
    State current_state_;

    // 地图点（用于 3D Gaussian Splatting）
    std::vector<MapGaussian> map_gaussian;

    // 优化滑动窗口
    void optimizeSlidingWindow();

    // 预积分IMU数据
    void preintegrateIMUData(const sensor_msgs::Imu::ConstPtr& imu_msg);

    // 提取图像特征并三角化地图点
    void extractImageFeatures(const sensor_msgs::Image::ConstPtr& image_msg);
};

} // namespace swgs_vio

#endif // SWGS_VIO_H