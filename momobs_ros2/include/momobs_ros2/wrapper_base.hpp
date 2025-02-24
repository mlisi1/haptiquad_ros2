#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <momobs/momentum_observer.hpp>
#include <momobs/force_estimator.hpp>

#include <std_msgs/msg/string.hpp>

#include <momobs_msgs/msg/residuals_stamped.hpp>
#include <momobs_msgs/msg/residual_error_stamped.hpp>
#include <momobs_msgs/msg/observer_gains.hpp>
#include <momobs_msgs/msg/estimated_forces.hpp>
#include <momobs_msgs/msg/friction_parameters.hpp>


#include <Eigen/Dense>
#include <vector>


class MomobsWrapperBase: public rclcpp::Node {

    public:

        MomobsWrapperBase();

    private:

        void descriptionCallback(const std_msgs::msg::String::SharedPtr msg);
        void gainsCallback(const momobs_msgs::msg::ObserverGains::SharedPtr msg);
        void frictionCallback(const momobs_msgs::msg::FrictionParameters::SharedPtr msg);


    protected:
    
        void publishResiduals();
        void publishForces();
        void publishResidualErrors();


    protected:

        bool description_received = false;
        bool first_message = true;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub;
        rclcpp::Subscription<momobs_msgs::msg::ObserverGains>::SharedPtr gains_sub;
        rclcpp::Subscription<momobs_msgs::msg::FrictionParameters>::SharedPtr friction_sub;


        rclcpp::Publisher<momobs_msgs::msg::ResidualsStamped>::SharedPtr residual_publisher;
        rclcpp::Publisher<momobs_msgs::msg::ResidualErrorStamped>::SharedPtr residual_error_publisher;
        rclcpp::Publisher<momobs_msgs::msg::EstimatedForces>::SharedPtr forces_publisher;

        momobs_msgs::msg::ResidualsStamped residual_msg;
        momobs_msgs::msg::EstimatedForces forces_msg;
        momobs_msgs::msg::ResidualErrorStamped residual_error_msg;

        std::map<std::string, double> msg_position_dict;
        std::map<std::string, double> msg_velocity_dict;
        std::map<std::string, double> msg_torques_dict;

        rclcpp::Time last_stamp, current_stamp;
        double dt;

        Eigen::VectorXd r_int, r_ext;
        Eigen::VectorXd err_int, err_ext;
        std::vector<Eigen::VectorXd> F;

        std::vector<std::string> joint_names, feet_frames;
        std::vector<Eigen::VectorXd> Forces;

        momobs::MomentumObserver observer;
        momobs::ForceEstimator estimator;


        //PARAMETERS
        //Time rescaling
        int num_contacts = 0;
        float k_int, k_ext = 1.0;
        bool rescale = false;
        double expected_dt = 0.0;
        double threshold = 0.0;  

        //Friction
        bool friction = false;
        double F_s    = 0.0;    // Static (stiction) friction torque [Nm]
        double F_c    = 0.0;    // Coulomb (dynamic) friction torque [Nm]
        double sigma0 = 0.0; // Stiffness coefficient [Nm/rad]
        double sigma1 = 0.0;  // Damping coefficient [Nm·s/rad]
        double sigma2 = 0.0;    // Viscous friction coefficient [Nm·s/rad]
        double alpha  = 0.0;    // Transition parameter [s/rad]
};