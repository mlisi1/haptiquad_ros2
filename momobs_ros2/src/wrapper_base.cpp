#include <momobs_ros2/wrapper_base.hpp>


MomobsWrapperBase::MomobsWrapperBase() : rclcpp::Node("momobs_ros2") {

    rclcpp::QoS qos(10); 
    qos.transient_local();

    description_sub = this->create_subscription<std_msgs::msg::String>(
        // "/robot_description",
        "/fb/floating_base_description",
        qos,
        std::bind(&MomobsWrapperBase::descriptionCallback, this, std::placeholders::_1)
    ); 

    gains_sub = this->create_subscription<momobs_msgs::msg::ObserverGains>(
        "~/gains",
        10,
        std::bind(&MomobsWrapperBase::gainsCallback, this, std::placeholders::_1)
    );

    residual_publisher = this->create_publisher<momobs_msgs::msg::ResidualsStamped>("~/residuals", 10);
    residual_error_publisher = this->create_publisher<momobs_msgs::msg::ResidualErrorStamped>("~/residual_errors", 10);
    forces_publisher = this->create_publisher<momobs_msgs::msg::EstimatedForces>("~/estimated_forces", 10);

    //PARAMETERS
    num_contacts = this->declare_parameter<int>("estimator.num_contacts", 0);
    k_int = this->declare_parameter<float>("observer.k_int", 1.0);
    k_ext = this->declare_parameter<float>("observer.k_ext", 1.0);
    rescale = this->declare_parameter<bool>("observer.rescale", false);
    expected_dt = this->declare_parameter<double>("observer.expected_dt", 0.0);
    threshold = this->declare_parameter<double>("observer.threshold", 0.0); 

}




void MomobsWrapperBase::descriptionCallback(const std_msgs::msg::String::SharedPtr msg) {

    if (description_received) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Description already received!");
        return;
    }

    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(msg->data, model);

    observer.initModel(model);
    observer.setInternalGain(k_int);
    observer.setExternalGain(k_ext);

    if (rescale) {
        observer.enableTimeScaling(expected_dt, threshold);
    }

    estimator.initModel(model);
    estimator.setNumContacts(num_contacts);

    residual_msg.r_int.resize(model.nv-6);
    residual_msg.r_ext.resize(6);
    residual_msg.names.resize(model.nv-6);


    residual_error_msg.err_int.resize(model.nv-6);
    residual_error_msg.err_ext.resize(6);
    residual_error_msg.names.resize(model.nv-6);


    forces_msg.forces.resize(num_contacts);
    forces_msg.names.resize(num_contacts);

    

    joint_names.resize(model.nv-6);

    for (int i=0; i<joint_names.size(); i++) {
        joint_names[i] = model.names[i+2];
    }

    residual_msg.names = joint_names;

    estimator.findFeetFrames(joint_names);

    feet_frames = estimator.getFeetFrames();

    for (int i=0; i<num_contacts; i++) {
        forces_msg.names[i].data = feet_frames[i];
    }

    description_received = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Description received!");

}



void MomobsWrapperBase::gainsCallback(const momobs_msgs::msg::ObserverGains::SharedPtr msg) {

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received gains");
    observer.setInternalGain(msg->k_int.data);
    observer.setExternalGain(msg->k_ext.data);

}




void MomobsWrapperBase::publishResiduals() {

    for (int i=0; i<r_int.size(); i++) {
        residual_msg.r_int[i] = r_int(i);
        
    }

    for (int i=0; i<r_ext.size(); i++) {
        residual_msg.r_ext[i] = r_ext(i);
    }

    residual_msg.stamp = current_stamp;
    residual_publisher->publish(residual_msg);

}




void MomobsWrapperBase::publishResidualErrors() {

    for (int i=0; i<err_int.size(); i++) {
        residual_error_msg.err_int[i] = err_int(i);
        
    }

    for (int i=0; i<err_ext.size(); i++) {
        residual_error_msg.err_ext[i] = err_ext(i);
    }

    residual_error_msg.stamp = current_stamp;
    residual_error_publisher->publish(residual_error_msg);

}





void MomobsWrapperBase::publishForces() {

    forces_msg.header.stamp = current_stamp;



    for (int i=0; i<num_contacts; i++) {

        forces_msg.forces[i].force.x =  F[i][0];
        forces_msg.forces[i].force.y =  F[i][1];
        forces_msg.forces[i].force.z =  F[i][2];
        forces_msg.forces[i].torque.x = F[i][3];
        forces_msg.forces[i].torque.y = F[i][4];
        forces_msg.forces[i].torque.z = F[i][5];


    }


    forces_publisher->publish(forces_msg);


}

