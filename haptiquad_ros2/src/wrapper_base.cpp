#include <haptiquad_ros2/wrapper_base.hpp>


HaptiQuadWrapperBase::HaptiQuadWrapperBase() : rclcpp::Node("haptiquad_ros2") {

    rclcpp::QoS qos(10); 
    qos.transient_local();

    description_sub = this->create_subscription<std_msgs::msg::String>(
        // "/robot_description",
        "/floating_base_description",
        qos,
        std::bind(&HaptiQuadWrapperBase::descriptionCallback, this, std::placeholders::_1)
    ); 

    gains_sub = this->create_subscription<haptiquad_msgs::msg::ObserverGains>(
        "~/gains",
        10,
        std::bind(&HaptiQuadWrapperBase::gainsCallback, this, std::placeholders::_1)
    );

    friction_sub = this->create_subscription<haptiquad_msgs::msg::FrictionParameters>(
        "~/friction",
        10,
        std::bind(&HaptiQuadWrapperBase::frictionCallback, this, std::placeholders::_1)
    );

    residual_publisher = this->create_publisher<haptiquad_msgs::msg::ResidualsStamped>("~/residuals", 10);
    residual_error_publisher = this->create_publisher<haptiquad_msgs::msg::ResidualErrorStamped>("~/residual_errors", 10);
    forces_publisher = this->create_publisher<haptiquad_msgs::msg::EstimatedForces>("~/estimated_forces", 10);

    //PARAMETERS
    num_contacts = this->declare_parameter<int>("estimator.num_contacts", 0);
    k_int = this->declare_parameter<float>("observer.k_int", 1.0);
    k_ext = this->declare_parameter<float>("observer.k_ext", 1.0);
    rescale = this->declare_parameter<bool>("observer.rescale", false);
    expected_dt = this->declare_parameter<double>("observer.expected_dt", 0.0);
    threshold = this->declare_parameter<double>("observer.threshold", 0.0); 
    base_link_name = this->declare_parameter<std::string>("estimator.base_link_name", "base");
    calculate_residual_error = this->declare_parameter<bool>("estimator.calculate_residual_error", false);


    friction    = this->declare_parameter<bool>("observer.friction.friction", false);
    F_s         = this->declare_parameter<double>("observer.friction.F_s", 0.0);    
    F_c         = this->declare_parameter<double>("observer.friction.F_c", 0.0);    

    observer.setFrictionParameters(friction, F_s, F_c); 
    estimator.setBaseFrame(base_link_name);


    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized base wrapper");

}




void HaptiQuadWrapperBase::descriptionCallback(const std_msgs::msg::String::SharedPtr msg) {

    if (description_received) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Description already received!");
        return;
    }

    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(msg->data, model);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received pinocchio model with " << model.nv << "DOFs");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "======================================================");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Model inertias:");
    for (size_t i=0; i<model.inertias.size(); i++) {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "\n" << model.inertias[i] << "\n");
        RCLCPP_DEBUG_STREAM(this->get_logger(), "----------------------------------");
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "=======================================================");

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


    forces_msg.forces.resize(num_contacts + 1);
    forces_msg.names.resize(num_contacts + 1);

    

    joint_names.resize(model.nv-6);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Joint names:");
    
    for (size_t i=0; i<joint_names.size(); i++) {
        joint_names[i] = model.names[i+2];
        RCLCPP_DEBUG_STREAM(this->get_logger(), joint_names[i]);
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "=======================================================");

    residual_msg.names = joint_names;
    residual_error_msg.names = joint_names;

    estimator.findFeetFrames(joint_names);

    feet_frames = estimator.getFeetFrames();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Feet frames:");
    for (int i=0; i<num_contacts; i++) {
        forces_msg.names[i].data = feet_frames[i];
        RCLCPP_DEBUG_STREAM(this->get_logger(), feet_frames[i]);
    }
    forces_msg.names[num_contacts].data = "base_wrench";
    RCLCPP_DEBUG_STREAM(this->get_logger(), "========================================================");

    description_received = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Description received!");

}



void HaptiQuadWrapperBase::gainsCallback(const haptiquad_msgs::msg::ObserverGains::SharedPtr msg) {

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received gains");
    observer.setInternalGain(msg->k_int.data);
    observer.setExternalGain(msg->k_ext.data);

}


void HaptiQuadWrapperBase::frictionCallback(const haptiquad_msgs::msg::FrictionParameters::SharedPtr msg) {

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received gains");
    observer.setFrictionParameters(msg->use_friction.data, msg->f_s.data, msg->f_c.data);
}




void HaptiQuadWrapperBase::publishResiduals() {

    for (int i=0; i<r_int.size(); i++) {
        residual_msg.r_int[i] = r_int(i);
        
    }

    for (int i=0; i<r_ext.size(); i++) {
        residual_msg.r_ext[i] = r_ext(i);
    }

    residual_msg.stamp = current_stamp;
    residual_publisher->publish(residual_msg);

}




void HaptiQuadWrapperBase::publishResidualErrors() {

    for (int i=0; i<err_int.size(); i++) {
        residual_error_msg.err_int[i] = err_int(i);
        
    }

    for (int i=0; i<err_ext.size(); i++) {
        residual_error_msg.err_ext[i] = err_ext(i);
    }

    residual_error_msg.stamp = current_stamp;
    residual_error_publisher->publish(residual_error_msg);

}





void HaptiQuadWrapperBase::publishForces() {

    forces_msg.header.stamp = current_stamp;
    forces_msg.header.frame_id = "world";

    for (int i=0; i<num_contacts; i++) {

        forces_msg.forces[i].force.x =  F[feet_frames[i]][0];
        forces_msg.forces[i].force.y =  F[feet_frames[i]][1];
        forces_msg.forces[i].force.z =  F[feet_frames[i]][2];
        forces_msg.forces[i].torque.x = F[feet_frames[i]][3];
        forces_msg.forces[i].torque.y = F[feet_frames[i]][4];
        forces_msg.forces[i].torque.z = F[feet_frames[i]][5];

    }

    forces_msg.forces[num_contacts].force.x = F["base_wrench"][0];
    forces_msg.forces[num_contacts].force.y = F["base_wrench"][1];
    forces_msg.forces[num_contacts].force.z = F["base_wrench"][2];
    forces_msg.forces[num_contacts].torque.x = F["base_wrench"][3];
    forces_msg.forces[num_contacts].torque.y = F["base_wrench"][4];
    forces_msg.forces[num_contacts].torque.z = F["base_wrench"][5];


    forces_publisher->publish(forces_msg);


}

