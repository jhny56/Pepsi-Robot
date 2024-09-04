#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_hardware_interfaces/action/gripper_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <memory>

class FindCanAction : public BT::AsyncActionNode
{
public:
    using GripperAction = robot_hardware_interfaces::action::GripperAction;
    using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;

    FindCanAction(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : BT::AsyncActionNode(name, config), node_(node), goal_sent_(false) 
    {
        this->client_ = rclcpp_action::create_client<GripperAction>(node_, "can_navigation");
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<bool>("find_can") };
    }

    BT::NodeStatus tick() override
    {
        if (!goal_sent_)
        {
            if (!this->client_->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
                return BT::NodeStatus::FAILURE;
            }

            auto goal_msg = GripperAction::Goal();

            goal_handle_future_ = this->client_->async_send_goal(goal_msg);
            goal_sent_ = true;
            RCLCPP_ERROR(node_->get_logger(), "Goal sent to Server");
        }

        if (goal_handle_future_.valid())
        {
            auto status = goal_handle_future_.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready)
            {
                auto goal_handle = goal_handle_future_.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server");
                    return BT::NodeStatus::FAILURE;
                }

                result_future_ = this->client_->async_get_result(goal_handle);
            }
            else
            {
                return BT::NodeStatus::RUNNING;
            }
        }

        if (result_future_.valid())
        {
            auto status = result_future_.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready)
            {
                auto result = result_future_.get();
                goal_sent_ = false;

                if (result.result->result)
                {
                    RCLCPP_INFO(node_->get_logger(), "Action succeeded");
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Action failed");
                    return BT::NodeStatus::FAILURE;
                }
            }
            else
            {
                return BT::NodeStatus::RUNNING;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<GripperAction>::SharedPtr client_;
    std::shared_future<typename GoalHandleGripperAction::SharedPtr> goal_handle_future_;
    std::shared_future<typename GoalHandleGripperAction::WrappedResult> result_future_;

    bool goal_sent_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("behavior_tree_node");

    BT::BehaviorTreeFactory factory;

    // Register the node with a lambda that captures the node pointer
    factory.registerBuilder<FindCanAction>(
        "FindCanAction",
        [&node](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<FindCanAction>(name, config, node);
        }
    );

    // Get the path to the XML file using ament_index_cpp
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_behavior_tree");
    std::string xml_file = package_share_directory + "/behavior_tree.xml";

    auto tree = factory.createTreeFromFile(xml_file);

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        // Tick the behavior tree
        tree.tickRoot();

        // Process ROS callbacks
        rclcpp::spin_some(node);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

