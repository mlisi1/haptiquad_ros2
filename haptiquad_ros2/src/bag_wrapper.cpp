#include <haptiquad_ros2/bag_wrapper.hpp>



BagWrapper::BagWrapper() : HaptiQuadWrapperBase() {

    bag_sub = this->create_subscription<anymal_msgs::msg::AnymalState>(
        "state_estimator/anymal_state",
        10,
        std::bind(&BagWrapper::bagCallback, this, std::placeholders::_1)
    );

}



void BagWrapper::bagCallback(const anymal_msgs::msg::AnymalState::SharedPtr msg) {

    if (!description_received) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot description was not yet received");
        return;
    }

    if (first_message) {

        last_stamp = msg->header.stamp; 
        first_message = false;
        return;

    }


    for (size_t i=0; i<msg->contacts.size(); i++) {

        Eigen::VectorXd tmp = Eigen::VectorXd::Zero(6);       

        tmp << msg->contacts[i].wrench.force.x,
                msg->contacts[i].wrench.force.y,
                msg->contacts[i].wrench.force.z,
                msg->contacts[i].wrench.torque.x,
                msg->contacts[i].wrench.torque.y,
                msg->contacts[i].wrench.torque.z;

        GT_F[msg->contacts[i].name] = tmp;

    }

    GT_F["base_wrench"] = Eigen::VectorXd::Zero(6);



    for (size_t i=0; i<msg->joints.position.size(); i++) {

        msg_position_dict[msg->joints.name[i]] =    msg->joints.position[i];
        msg_velocity_dict[msg->joints.name[i]] =    msg->joints.velocity[i];
        msg_torques_dict[msg->joints.name[i]] =     msg->joints.effort[i];
        
    }

    observer.updateJointStates(msg_position_dict, msg_velocity_dict, msg_torques_dict);

    Eigen::Quaterniond orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                    msg->pose.pose.orientation.x,
                                                    msg->pose.pose.orientation.y,
                                                    msg->pose.pose.orientation.z);

    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(6);
    v0 << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z; 


    observer.updateBaseState(v0, orientation);

    current_stamp = rclcpp::Time(msg->header.stamp, last_stamp.get_clock_type());
    dt = (current_stamp - last_stamp).seconds();

    std::tie(r_int, r_ext) = observer.getResiduals(dt);

    publishResiduals();

    estimator.updateJacobians(msg_position_dict, observer.getF(), observer.getIC());

    F = estimator.calculateForces(r_int, r_ext, orientation);

    std::tie(gt_r_int, gt_r_ext) = estimator.calculateResidualsFromForces(GT_F);
    err_int = gt_r_int - r_int;
    err_ext = gt_r_ext - r_ext;

    publishForces();
    publishResidualErrors();

    last_stamp = msg->header.stamp;


}















int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<BagWrapper>();
    rclcpp::spin(node);
}