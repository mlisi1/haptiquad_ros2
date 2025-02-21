#include <momobs_ros2/bag_wrapper.hpp>



BagWrapper::BagWrapper() : MomobsWrapperBase() {

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


    for (int i=0; i<msg->joints.position.size(); i++) {

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

    current_stamp = rclcpp::Time(msg->header.stamp);
    dt = (current_stamp - last_stamp).seconds();

    std::tie(r_int, r_ext) = observer.getResiduals(dt);

    publishResiduals();

    std::map<std::string, bool> is_on_ground;

    for (int i=0; i<msg->contacts.size(); i++) {
        is_on_ground[msg->contacts[i].name] = msg->contacts[i].state;
    }

    estimator.setFeetOnGround(is_on_ground);
    estimator.updateJacobians(msg_position_dict, observer.getF(), observer.getIC());

    F = estimator.calculateForces(r_int, r_ext, orientation);

    publishForces();

    last_stamp = msg->header.stamp;


}















int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<BagWrapper>();
    rclcpp::spin(node);
}