#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>

using namespace Eigen;

struct DHParameter {
    double theta;  // joint angle
    double alpha;  // link twist
    double a;      // link length
    double d;      // link offset
};

Matrix4d dh_transform(double theta, double alpha, double a, double d) {
    Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

Matrix4d forward_kinematics(const std::vector<DHParameter>& dh_params) {
    Matrix4d T = Matrix4d::Identity();
    for (const auto& param : dh_params) {
        T = T * dh_transform(param.theta, param.alpha, param.a, param.d);
    }
    return T;
}

class ForwardKinematicNode : public rclcpp::Node {
public:
    // Constructor
    ForwardKinematicNode() : Node("forward_kinematic_node") {
        RCLCPP_INFO(this->get_logger(), "Starting ForwardKinematicNode...");
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ForwardKinematicNode::topic_callback, this, std::placeholders::_1));
    }

    // Callback function
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> joint_angles;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_1" || msg->name[i] == "joint_2" || msg->name[i] == "joint_3" || 
                msg->name[i] == "joint_4" || msg->name[i] == "joint_5" || msg->name[i] == "joint_6" || 
                msg->name[i] == "joint_7") {
                joint_angles.push_back(msg->position[i]);
                RCLCPP_INFO(this->get_logger(), "Parsed angle for %s: %f", msg->name[i].c_str(), msg->position[i]);
            }
        }

        if (joint_angles.size() == 7) {
            std::vector<DHParameter> dh_params = {
                {0, M_PI, 0.0, 0.0},
                {joint_angles[0], M_PI_2, 0.0, -(0.1564 + 0.1284)},
                {joint_angles[1] + M_PI, M_PI_2, 0.0, -(0.0054 + 0.0064)},
                {joint_angles[2] + M_PI, M_PI_2, 0.0, -(0.2104 + 0.2104)},
                {joint_angles[3] + M_PI, M_PI_2, 0.0, -(0.0064 + 0.0064)},
                {joint_angles[4] + M_PI, M_PI_2, 0.0, -(0.2084 + 0.1059)},
                {joint_angles[5] + M_PI, M_PI_2, 0.0, 0.0},
                {joint_angles[6] + M_PI, M_PI, 0.0, -(0.1059 + 0.0615)}
            };

            Matrix4d T = forward_kinematics(dh_params);

            // Create a stringstream to format the matrix output
            std::ostringstream oss;
            oss << "Forward Kinematics Transformation Matrix:\n" << T;
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Expected 7 joint angles, but got %zu", joint_angles.size());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForwardKinematicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
