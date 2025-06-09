#include <haptiquad_ros2/mujoco_wrapper.hpp>



MujocoWrapper::MujocoWrapper() : HaptiQuadWrapperBase() {

    rmw_qos_profile_t mujoco_qos = rmw_qos_profile_sensor_data;
    mujoco_qos.depth = 1;

    joint_state_sub_.subscribe(this, "/simulation/joint_states", mujoco_qos);
    odom_sub_.subscribe(this, "/simulation/sensor_odom", mujoco_qos);
    mujoco_contacts_sub_.subscribe(this, "/simulation/contacts", mujoco_qos);
    mujoco_wrench_sub_.subscribe(this, "/simulation/base_wrench", mujoco_qos);

    mujoco_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), 
        joint_state_sub_, 
        odom_sub_
    );

    mujoco_gt_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicyGT>>(
        SyncPolicyGT(10), 
        mujoco_wrench_sub_,
        mujoco_contacts_sub_
    );

    mujoco_sync_->registerCallback(std::bind(&MujocoWrapper::mujocoCallback, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2)
    );


    mujoco_gt_sync_->registerCallback(std::bind(&MujocoWrapper::mujocoGTCallback, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2)
    );

}



void MujocoWrapper::mujocoCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state,
                                    const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {

    if (!description_received) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot description was not yet received");
        return;
    }

    if (first_message) {

        last_stamp = joint_state->header.stamp; 
        first_message = false;
        return;

    }


    for (size_t i=0; i<joint_state->position.size(); i++) {

        msg_position_dict[joint_state->name[i]] =    joint_state->position[i];
        msg_velocity_dict[joint_state->name[i]] =    joint_state->velocity[i];
        msg_torques_dict[joint_state->name[i]] =     joint_state->effort[i];
        
    }

    observer.updateJointStates(msg_position_dict, msg_velocity_dict, msg_torques_dict);

    Eigen::Quaterniond orientation = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                    odom->pose.pose.orientation.x,
                                                    odom->pose.pose.orientation.y,
                                                    odom->pose.pose.orientation.z);

    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(6);
    v0 << odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            odom->twist.twist.linear.z,
            odom->twist.twist.angular.x,
            odom->twist.twist.angular.y,
            odom->twist.twist.angular.z; 


    observer.updateBaseState(v0, orientation);

    current_stamp = rclcpp::Time(joint_state->header.stamp);
    dt = (current_stamp - last_stamp).seconds();

    std::tie(r_int, r_ext) = observer.getResiduals(dt);

    publishResiduals();

    estimator.updateJacobians(msg_position_dict, observer.getF(), observer.getIC());

    F = estimator.calculateForces(r_int, r_ext, orientation);

    publishForces();

    last_stamp = joint_state->header.stamp;


}





void MujocoWrapper::mujocoGTCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &base_wrench,
                            const mujoco_msgs::msg::MujocoContacts::ConstSharedPtr &contacts) {

     if (!description_received) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot description was not yet received");
        return;
    }


    current_stamp = rclcpp::Time(base_wrench->header.stamp);

    for (int i=0; i<num_contacts; i++) {
        GT_F[feet_frames[i]] = Eigen::VectorXd::Zero(6);
    }


    for (size_t i=0; i<contacts->contacts.size(); i++) {
        std::string contact_name = contacts->contacts[i].object2_name;
        for (int j=0; j<num_contacts; j++) {
            if (contact_name == feet_frames[j]) {
                GT_F[contact_name] <<   contacts->contacts[i].contact_force.force.x,
                                        contacts->contacts[i].contact_force.force.y,
                                        contacts->contacts[i].contact_force.force.z,
                                        contacts->contacts[i].contact_force.torque.x,
                                        contacts->contacts[i].contact_force.torque.y,
                                        contacts->contacts[i].contact_force.torque.z;
            }
        }

    }


    if (current_stamp != gt_stamp) {

        RCLCPP_WARN_STREAM(this->get_logger(), "Time mismatch between estimation and real values");

    }


    std::tie(gt_r_int, gt_r_ext) = estimator.calculateResidualsFromForces(GT_F);
    err_int = gt_r_int - r_int;
    err_ext = gt_r_ext - r_ext;

    publishResidualErrors();



}















int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MujocoWrapper>();
    rclcpp::spin(node);
}