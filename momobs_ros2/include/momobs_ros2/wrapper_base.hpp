#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <momobs/momentum_observer.hpp>
#include <momobs/force_estimator.hpp>

#include <std_msgs/msg/string.hpp>

#include <observer_msgs/msg/residuals_stamped.hpp>
#include <observer_msgs/msg/observer_gains.hpp>
#include <observer_msgs/msg/estimation_errors.hpp>
#include <observer_msgs/msg/estimated_forces.hpp>

#include <Eigen/Dense>
#include <vector>


class MomobsWrapperBase: public rclcpp::Node {

    public:

        MomobsWrapperBase();

    private:

        void descriptionCallback(const std_msgs::msg::String::SharedPtr msg);
        void gainsCallback(const observer_msgs::msg::ObserverGains::SharedPtr msg);

    protected:
    
        void publishResiduals();
        void publishForces();


    protected:

        bool description_received = false;
        bool first_message = true;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub;
        rclcpp::Subscription<observer_msgs::msg::ObserverGains>::SharedPtr gains_sub;

        rclcpp::Publisher<observer_msgs::msg::ResidualsStamped>::SharedPtr residual_publisher;
        rclcpp::Publisher<observer_msgs::msg::EstimatedForces>::SharedPtr forces_publisher;

        observer_msgs::msg::ResidualsStamped residual_msg;
        observer_msgs::msg::EstimatedForces forces_msg;

        std::map<std::string, double> msg_position_dict;
        std::map<std::string, double> msg_velocity_dict;
        std::map<std::string, double> msg_torques_dict;

        rclcpp::Time last_stamp, current_stamp;
        double dt;

        Eigen::VectorXd r_int, r_ext;
        std::vector<Eigen::VectorXd> F;

        std::vector<std::string> joint_names;
        std::vector<Eigen::VectorXd> Forces;

        momobs::MomentumObserver observer;
        momobs::ForceEstimator estimator;


        //PARAMETERS
        int num_contacts = 0;
        float k_int, k_ext = 1.0;
        bool rescale = false;
        double expected_dt = 0.0;
        double threshold = 0.0;  
};