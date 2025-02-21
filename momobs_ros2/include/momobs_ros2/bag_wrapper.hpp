#include <momobs_ros2/wrapper_base.hpp>

#include <anymal_msgs/msg/anymal_state.hpp>





class BagWrapper : public MomobsWrapperBase {

    public:

        BagWrapper();

        void bagCallback(const anymal_msgs::msg::AnymalState::SharedPtr msg);


    private:

        rclcpp::Subscription<anymal_msgs::msg::AnymalState>::SharedPtr bag_sub;

        



};