#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <ruckig/ruckig.hpp>
#include "tm_kinematics/tm_kin.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetVelocity.h"
#include "tf2/utils.h"

#define NUM_DOF 6

namespace control_interface {
    class ruckig_controller {
public:
        ruckig_controller(ros::NodeHandle& nh, double ros_rate)
        {
            this->n = nh;
            joint_sub = n.subscribe("/joint_states", 1, &ruckig_controller::joint_callback, this);
            target_sub = n.subscribe("/target_position", 1, &ruckig_controller::target_callback, this);
            target_joint_sub = n.subscribe("/target_joint", 1, &ruckig_controller::target_joint_callback, this);
            vel_client = n.serviceClient < tm_msgs::SetVelocity > ("tm_driver/set_velocity");
            script_client = n.serviceClient < tm_msgs::SendScript > ("tm_driver/send_script");
            max_velocities.fill(0.08);
            max_accelerations.fill(1.0);
            max_jerks.fill(3.0);
            ruckig_timestep = 1 / ros_rate;
            std::fill(&tm_target_j[0], &tm_target_j[0] + 6, 0.0);
            std::fill(&tm_target_p[0], &tm_target_p[0] + 6, 0.0);
        }
        void run();
        void script_call();

private:
        ros::NodeHandle n;
        ros::Subscriber joint_sub;
        ros::Subscriber target_sub;
        ros::Subscriber target_joint_sub;
        ros::ServiceClient script_client;
        ros::ServiceClient vel_client;
        ruckig::InputParameter < ruckig::DynamicDOFs > ruckig_input {NUM_DOF};
        ruckig::OutputParameter < ruckig::DynamicDOFs > ruckig_output {NUM_DOF};
        std::unique_ptr < ruckig::Ruckig < ruckig::DynamicDOFs >> ruckig_ptr;
        std::array < double, NUM_DOF > max_velocities, max_accelerations, max_jerks;
        std::array < double, NUM_DOF > current_positions, current_velocities, current_accelerations;
        double ruckig_timestep;
        double tm_target_p[6];
        double tm_target_j[6];

        void ruckig_state_manage();
        void joint_callback(const sensor_msgs::JointState& joint_msg);
        void target_callback(const geometry_msgs::TransformStamped& target_msg);
        void target_joint_callback(const sensor_msgs::JointState& target_joint_msg);
        void ruckig_stop();
        void getNextRuckigInput();
        void initializeRuckigState();
        bool ruckig_activate = false;
    };
}
