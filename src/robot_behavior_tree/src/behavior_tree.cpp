#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_hardware_interfaces/action/gripper_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <memory>
#include "std_msgs/msg/bool.hpp"

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
            auto status = goal_handle_future_.wait_for(std::chrono::seconds(1));
            if (status == std::future_status::ready)
            {
                auto goal_handle = goal_handle_future_.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server");
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Goal was accepted by the server");
                }

                // Request the result
                result_future_ = this->client_->async_get_result(goal_handle);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Waiting for goal acceptance...");
                return BT::NodeStatus::RUNNING;
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
            return BT::NodeStatus::FAILURE;
        }

        if (result_future_.valid())
        {
            auto status = result_future_.wait_for(std::chrono::seconds(60));
            if (status == std::future_status::ready)
            {
                auto result = result_future_.get();
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
                RCLCPP_INFO(node_->get_logger(), "Waiting for result...");
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

class FindQrCodeAction : public BT::AsyncActionNode
{
public:
    using GripperAction = robot_hardware_interfaces::action::GripperAction;
    using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;

    FindQrCodeAction(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : BT::AsyncActionNode(name, config), node_(node) 
    {
        // Implementation to search for QR code
        this->client_ = rclcpp_action::create_client<GripperAction>(node_, "qr_navigation");
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<bool>("find_qr") };
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
            auto status = result_future_.wait_for(std::chrono::seconds(60));
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


class IsAtCanCondition : public BT::ConditionNode
{
public:
    IsAtCanCondition(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : BT::ConditionNode(name, config), node_(node)
    {
        // Subscribe to the "/is_at_can" topic or any other topic that indicates whether the robot is at the can.
        subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/is_at_can", 10, std::bind(&IsAtCanCondition::callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // If the robot is at the can, return SUCCESS, otherwise return FAILURE
        if (is_at_can_)
        {
            RCLCPP_ERROR(node_->get_logger(), "ROBOT is at can");

            return BT::NodeStatus::SUCCESS;
        }
        
        RCLCPP_ERROR(node_->get_logger(), "ROBOT is not at can");

        return BT::NodeStatus::FAILURE;
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_at_can_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    bool is_at_can_ = false;  
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("behavior_tree_node");

    BT::BehaviorTreeFactory factory;

    // Register FindCanAction
    factory.registerBuilder<FindCanAction>(
        "FindCanAction",
        [&node](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<FindCanAction>(name, config, node);
        }
    );

    // Register IsAtCanCondition
    factory.registerBuilder<IsAtCanCondition>(
        "IsAtCanCondition",
        [&node](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<IsAtCanCondition>(name, config, node);
        }
    );

    // Register FindQrCodeAction
    factory.registerBuilder<FindQrCodeAction>(
        "FindQrCodeAction",
        [&node](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<FindQrCodeAction>(name, config, node);
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
