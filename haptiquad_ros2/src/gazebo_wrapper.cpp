#include <haptiquad_ros2/gazebo_wrapper.hpp>



GazeboWrapper::GazeboWrapper() : HaptiQuadWrapperBase() {

    joint_state_sub_.subscribe(this, "/joint_states");
    odom_sub_.subscribe(this, "/base");
    LF_contact_sub_.subscribe(this, "/contact_force_sensors/LF");
    RF_contact_sub_.subscribe(this, "/contact_force_sensors/RF");
    LH_contact_sub_.subscribe(this, "/contact_force_sensors/LH");
    RH_contact_sub_.subscribe(this, "/contact_force_sensors/RH");
    base_wrench_sub_.subscribe(this, "/wrench");

    gazebo_gt_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicyGT>>(
        SyncPolicyGT(10), 
        joint_state_sub_, 
        odom_sub_, 
        LF_contact_sub_,
        RF_contact_sub_,
        LH_contact_sub_,
        RH_contact_sub_,
        base_wrench_sub_
    );

    gazebo_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), 
        joint_state_sub_, 
        odom_sub_
    );


    if (calculate_residual_error) {

        gazebo_gt_sync_->registerCallback(std::bind(&GazeboWrapper::gazeboGTCallback, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2, 
                                    std::placeholders::_3,
                                    std::placeholders::_4,
                                    std::placeholders::_5,
                                    std::placeholders::_6,
                                    std::placeholders::_7)
        );

    } else {

        gazebo_sync_->registerCallback(std::bind(&GazeboWrapper::gazeboCallback, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2)
        );

    }

    

}


void GazeboWrapper::gazeboCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state,
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


    current_stamp = rclcpp::Time(joint_state->header.stamp, last_stamp.get_clock_type());
    dt = (current_stamp - last_stamp).seconds();

    std::tie(r_int, r_ext) = observer.getResiduals(dt);

    publishResiduals();

    estimator.updateJacobians(msg_position_dict, observer.getF(), observer.getIC());

    F = estimator.calculateForces(r_int, r_ext, orientation);

    publishForces();

    last_stamp = current_stamp;



}



 void GazeboWrapper::gazeboGTCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state,
                            const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &LF_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &RF_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &LH_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &RH_contact,
                            const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &base_wrench) {

    

    if (!description_received) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot description was not yet received");
        return;
    }

    if (gt_first_message) {
        gt_first_message = false;
        return;
    }


    gazeboCallback(joint_state, odom);


    for (int i=0; i<num_contacts; i++) {
        GT_F[feet_frames[i]] = Eigen::VectorXd::Zero(6);
    }

    GT_F["base_wrench"] = Eigen::VectorXd::Zero(6);

    if (!LF_contact->states.size()==0) {
        GT_F["LF_FOOT"] <<  LF_contact->states.back().total_wrench.force.x,
                            LF_contact->states.back().total_wrench.force.y,
                            LF_contact->states.back().total_wrench.force.z,
                            LF_contact->states.back().total_wrench.torque.x,
                            LF_contact->states.back().total_wrench.torque.y,
                            LF_contact->states.back().total_wrench.torque.z;
    }

    if (!RF_contact->states.size()==0) {
        GT_F["RF_FOOT"] <<  RF_contact->states.back().total_wrench.force.x,
                            RF_contact->states.back().total_wrench.force.y,
                            RF_contact->states.back().total_wrench.force.z,
                            RF_contact->states.back().total_wrench.torque.x,
                            RF_contact->states.back().total_wrench.torque.y,
                            RF_contact->states.back().total_wrench.torque.z;
    }

    if (!LH_contact->states.size()==0) {
        GT_F["LH_FOOT"] <<  LH_contact->states.back().total_wrench.force.x,
                            LH_contact->states.back().total_wrench.force.y,
                            LH_contact->states.back().total_wrench.force.z,
                            LH_contact->states.back().total_wrench.torque.x,
                            LH_contact->states.back().total_wrench.torque.y,
                            LH_contact->states.back().total_wrench.torque.z;
    }

    if (!RH_contact->states.size()==0) {
        GT_F["RH_FOOT"] <<  RH_contact->states.back().total_wrench.force.x,
                            RH_contact->states.back().total_wrench.force.y,
                            RH_contact->states.back().total_wrench.force.z,
                            RH_contact->states.back().total_wrench.torque.x,
                            RH_contact->states.back().total_wrench.torque.y,
                            RH_contact->states.back().total_wrench.torque.z;
    }


    GT_F["base_wrench"] << base_wrench->wrench.force.x,
                            base_wrench->wrench.force.y,
                            base_wrench->wrench.force.z,
                            base_wrench->wrench.torque.x,
                            base_wrench->wrench.torque.y,
                            base_wrench->wrench.torque.z;


    std::tie(gt_r_int, gt_r_ext) = estimator.calculateResidualsFromForces(GT_F);

    err_int = gt_r_int - r_int;
    err_ext = gt_r_ext - r_ext;

    publishResidualErrors();

    
}














int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<GazeboWrapper>();
    rclcpp::spin(node);
}
