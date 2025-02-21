#include <momobs_ros2/wrapper_base.hpp>

#include <mujoco_msgs/msg/mujoco_contacts.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>


using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::JointState, nav_msgs::msg::Odometry, mujoco_msgs::msg::MujocoContacts>;


class MujocoWrapper : public MomobsWrapperBase {

    public:

        MujocoWrapper();

        void mujocoCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state,
                            const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                            const mujoco_msgs::msg::MujocoContacts::ConstSharedPtr &contacts);


    private:

        message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        message_filters::Subscriber<mujoco_msgs::msg::MujocoContacts> mujoco_contacts_sub_;
                
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> mujoco_sync_;       



};