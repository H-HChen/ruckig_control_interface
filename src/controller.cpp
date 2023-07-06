#include "controller.h"

using namespace control_interface;


bool CheckJointLimit(double *q)
{
    bool valid = true;

    if (abs(q[0]) > 265 * DEG2RAD) {
        ROS_WARN("[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
        valid = false;
    } else if (abs(q[1]) > 175 * DEG2RAD) {
        ROS_WARN("[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
        valid = false;
    } else if (abs(q[2]) > 148 * DEG2RAD) {
        ROS_WARN("[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
        valid = false;
    } else if (abs(q[3]) > 175 * DEG2RAD) {
        ROS_WARN("[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
        valid = false;
    } else if (abs(q[4]) > 175 * DEG2RAD) {
        ROS_WARN("[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
        valid = false;
    } else if (abs(q[5]) > 265 * DEG2RAD) {
        ROS_WARN("[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
        valid = false;
    } else {
        valid = true;
    }
    return valid;
}

bool GetQfromInverseKinematics(double* CartesianPosition, double *q_inv)
{
    Eigen::Matrix < float, 4, 4 > T_;
    Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
    Eigen::Quaternion < float > q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix < float, 3, 3 > RotationMatrix = q.matrix();
    double *T = new double[16];


    T_ << 0., 0., 0., CartesianPosition[0],
        0., 0., 0., CartesianPosition[1],
        0., 0., 0., CartesianPosition[2],
        0., 0., 0., 1.;

    T_.block < 3, 3 > (0, 0) = RotationMatrix.block < 3, 3 > (0, 0);

    tm_jacobian::Matrix2DoubleArray(T_, T);
    int num_sol = tm_kinematics::inverse(T, q_inv);

    delete[] T;
    return CheckJointLimit(q_inv);
}

void ruckig_controller::getNextRuckigInput()
{
    bool within_limit = true;

    if (std::count(tm_target_p, tm_target_p + 6, 0.0) != 6) {
        within_limit = GetQfromInverseKinematics(tm_target_p, tm_target_j);
    }
    if (within_limit) {
        for (size_t joint = 0; joint < NUM_DOF; ++joint) {
            // Target state is the next waypoint
            ruckig_input.target_position.at(joint) = tm_target_j[joint];
            ruckig_input.target_velocity.at(joint) = 0.0;
            ruckig_input.target_acceleration.at(joint) = 0.0;
        }
        initializeRuckigState();
        ROS_INFO("Recieve New Target Point!!");
    } else {
        ROS_WARN("Joint position over limit, skip target");
    }
}

void ruckig_controller::ruckig_state_manage()
{
    if (ruckig_activate) {
        ruckig::Result ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
        if (ruckig_result != ruckig::Result::Finished) {
            auto& p = ruckig_output.new_velocity;
            tm_msgs::SetVelocity vel_srv;
            std::vector < double > cmd_vel(NUM_DOF);
            for (size_t i = 0; i < NUM_DOF; ++i) {
                cmd_vel.at(i) = p.at(i);
                current_accelerations.at(i) = std::clamp(
                    ruckig_output.new_acceleration.at(i), -ruckig_input.max_acceleration.at(i), ruckig_input.max_acceleration.at(i));
            }
            vel_srv.request.motion_type = 1;
            vel_srv.request.velocity = cmd_vel;
            vel_client.call(vel_srv);
            ruckig_output.pass_to_input(ruckig_input);
        } else {
            ROS_INFO("Target Reached!!");
            ruckig_activate = false;
            tm_msgs::SetVelocity vel_srv;
            std::vector < double > cmd_vel(NUM_DOF, 0.0);
            vel_srv.request.motion_type = 1;
            vel_srv.request.velocity = cmd_vel;
            vel_client.call(vel_srv);
            std::fill(&tm_target_j[0], &tm_target_j[0] + 6, 0.0);
            std::fill(&tm_target_p[0], &tm_target_p[0] + 6, 0.0);
        }
    }
}

void ruckig_controller::joint_callback(const sensor_msgs::JointState& joint_msg)
{
    for (size_t i = 0; i < NUM_DOF; ++i) {
        current_velocities.at(i) =
            std::clamp(joint_msg.velocity.at(i), -ruckig_input.max_velocity.at(i), ruckig_input.max_velocity.at(i));
        current_positions.at(i) = joint_msg.position.at(i);
    }
    if (!ruckig_activate) {
        current_accelerations.fill(0.0);
    }
}

void ruckig_controller::target_joint_callback(const sensor_msgs::JointState& target_joint_msg)
{
    ruckig_ptr = std::make_unique < ruckig::Ruckig < ruckig::DynamicDOFs >> (NUM_DOF, ruckig_timestep);
    for (size_t joint = 0; joint < NUM_DOF; ++joint) {
        tm_target_j[joint] = target_joint_msg.position.at(joint);
    }

    getNextRuckigInput();
    ruckig_activate = true;
}

void ruckig_controller::target_callback(const geometry_msgs::TransformStamped& target_msg)
{
    ruckig_ptr = std::make_unique < ruckig::Ruckig < ruckig::DynamicDOFs >> (NUM_DOF, ruckig_timestep);
    tm_target_p[0] = target_msg.transform.translation.x;
    tm_target_p[1] = target_msg.transform.translation.y;
    tm_target_p[2] = target_msg.transform.translation.z;
    tf2::Quaternion quat;
    tf2::fromMsg(target_msg.transform.rotation, quat);
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

    getNextRuckigInput();
    ruckig_activate = true;
}

void ruckig_controller::ruckig_stop()
{
    ruckig_input.control_interface = ruckig::ControlInterface::Velocity;
    ruckig_input.synchronization = ruckig::Synchronization::None;
    ruckig_input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ruckig_input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void ruckig_controller::initializeRuckigState()
{
    ruckig_input.control_interface = ruckig::ControlInterface::Position;
    std::copy_n(current_positions.begin(), NUM_DOF, ruckig_input.current_position.begin());
    std::copy_n(current_velocities.begin(), NUM_DOF, ruckig_input.current_velocity.begin());
    std::copy_n(current_accelerations.begin(), NUM_DOF, ruckig_input.current_acceleration.begin());
    std::copy_n(max_velocities.begin(), NUM_DOF, ruckig_input.max_velocity.begin());
    std::copy_n(max_accelerations.begin(), NUM_DOF, ruckig_input.max_acceleration.begin());
    std::copy_n(max_jerks.begin(), NUM_DOF, ruckig_input.max_jerk.begin());
    // Initialize output data struct
    ruckig_output.new_position = ruckig_input.current_position;
    ruckig_output.new_velocity = ruckig_input.current_velocity;
    ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

void ruckig_controller::script_call()
{
    tm_msgs::SendScript script_srv;

    script_srv.request.id = "Vstart";
    script_srv.request.script = "ContinueVJog()";
    script_client.call(script_srv);
    ROS_INFO("Sending script service request to tm_driver...");
}

void ruckig_controller::run()
{
    ruckig_state_manage();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;
    double ros_rate = 25.0;
    auto controller_node = std::make_shared < control_interface::ruckig_controller > (n, ros_rate);
    ros::Rate rate(ros_rate);
    controller_node->script_call();
    ROS_INFO("Controller Node Startup");
    while (ros::ok()) {
        controller_node->run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
