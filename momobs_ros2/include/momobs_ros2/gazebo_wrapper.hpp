#include <momobs_ros2/wrapper_base.hpp>


#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include <gazebo_msgs/msg/contacts_state.hpp>





using SyncPolicy = message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::JointState, 
                nav_msgs::msg::Odometry, 
                gazebo_msgs::msg::ContactsState,
                gazebo_msgs::msg::ContactsState,
                gazebo_msgs::msg::ContactsState,
                gazebo_msgs::msg::ContactsState>;


class GazeboWrapper : public MomobsWrapperBase {

    public:

        GazeboWrapper();

        void gazeboCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state,
                            const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &LF_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &RF_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &LH_contact,
                            const gazebo_msgs::msg::ContactsState::ConstSharedPtr &RH_contact);


    private:

        message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        message_filters::Subscriber<gazebo_msgs::msg::ContactsState> LF_contact_sub_;
        message_filters::Subscriber<gazebo_msgs::msg::ContactsState> RF_contact_sub_;
        message_filters::Subscriber<gazebo_msgs::msg::ContactsState> LH_contact_sub_;
        message_filters::Subscriber<gazebo_msgs::msg::ContactsState> RH_contact_sub_;
                
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> gazebo_sync_;       



};