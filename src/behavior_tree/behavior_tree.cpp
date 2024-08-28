#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <stdexcept>
#include "control/gripper_action_client.hpp"

using namespace std::chrono_literals;

/*Can subtree*/
BT::NodeStatus foundCan(BT::TreeNode &self){
    BT::Optional<bool> msg = self.getInput<bool>("canfound");
    if(!msg){
        throw BT::RuntimeError("No canfound message: ", msg.error());
    }
     
    if (msg == true){
        std::cout << "Can found" << std::endl;       
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Can not found" << std::endl;
    return BT::NodeStatus::FAILURE;
}

class SearchCan : public BT::SyncActionNode
{
public:
    explicit SearchCan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts(){
        return {BT::BidirectionalPort<bool>("process_started")};
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<bool> msg = getInput<bool>("process_started");
        if (msg == false) {
            try {
                // Start the Python node in the background
                std::string command = "ros2 run perception pepsican_detector &";
                std::system(command.c_str());
                
                bool started = true;
                BT::TreeNode::setOutput("process_started", started);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("SearchCan"), "Exception: %s", e.what());
                return BT::NodeStatus::FAILURE;
            }
        }

        // Indicate success immediately after starting the node
        return BT::NodeStatus::SUCCESS;
    }
};



BT::NodeStatus isAtCan(){
    bool atCan = false;   
    if (atCan){
        std::cout << "Robot is at can. Close gripper" << std::endl;
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Robot not at can." << std::endl;
    return BT::NodeStatus::FAILURE;
}

class GoToCan : public BT::SyncActionNode
{
public:
    explicit GoToCan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) 
    {
    }

    static BT::PortsList providedPorts(){
        return {BT::BidirectionalPort<bool>("can_nav")};
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<bool> msg = getInput<bool>("can_nav");
        if (msg == false) {
            try {
                // Start the Python node
                std::string command = "ros2 run navigation can_navigation_node";
                int result = std::system(command.c_str());

                if (result != 0) {
                    throw std::runtime_error("Failed to start CAN NAVIGATION node.");
                }
                BT::TreeNode::setOutput("can_nav", true);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("GoToCan"), "Exception: %s", e.what());
                return BT::NodeStatus::FAILURE;
            }
        }

        // You can add additional logic here to check the status of the node if needed
        return BT::NodeStatus::SUCCESS;
    }

};


class Gripper : public BT::StatefulActionNode
{
public:
    explicit Gripper(const std::string &name)
        : BT::StatefulActionNode(name, {}), 
          node_(std::make_shared<GripperActionClient>()), goal_active_(false) 
    {
        rclcpp::spin_some(node_);
    }

    BT::NodeStatus onStart() override
    {
        // Start the action client by sending the goal
        node_->send_goal();
        goal_active_ = true;

        // Process any immediate ROS callbacks
        rclcpp::spin_some(node_);

        // Indicate that the action is running
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Keep spinning to handle ROS callbacks
        rclcpp::spin_some(node_);

        // Check if the goal is still active
        if (!goal_active_)
        {
            // If the goal is no longer active, determine success or failure
            return BT::NodeStatus::SUCCESS;
        }

        // Otherwise, keep running
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // Handle any necessary cleanup if the action is halted
        goal_active_ = false;
    }

private:
    std::shared_ptr<GripperActionClient> node_;
    bool goal_active_;
};

/*QR subtree*/
BT::NodeStatus foundQr(BT::TreeNode &self){
    BT::Optional<bool> msg = self.getInput<bool>("QRfound");
    if(!msg){
        throw BT::RuntimeError("No QRfound message: ", msg.error());
    }
     
    if (msg == true){
        std::cout << "Robot found QR" << std::endl;       
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Robot did not found QR" << std::endl;
    return BT::NodeStatus::FAILURE;
}

class SearchQr : public BT::SyncActionNode
{
public:
    explicit SearchQr(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts(){
        return {BT::BidirectionalPort<bool>("qr_process_started")};
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<bool> msg = getInput<bool>("qr_process_started");
        if (msg == false) {
            try {
                // Start the Python node in the background
                std::string command = "ros2 run perception qrcode_detector &";
                std::system(command.c_str());
                
                bool started = true;
                BT::TreeNode::setOutput("qr_process_started", started);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("SearchQr"), "Exception: %s", e.what());
                return BT::NodeStatus::FAILURE;
            }
        }

        // Indicate success immediately after starting the node
        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus isAtQr(){
    bool atQR = false;   
    if (atQR){
        std::cout << "Robot is at QR code" << std::endl;
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Robot is not at QR code." << std::endl;
    return BT::NodeStatus::FAILURE;
}

class GoToQr : public BT::SyncActionNode
{
public:
    explicit GoToQr(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) 
    {
    }

    static BT::PortsList providedPorts(){
        return {BT::BidirectionalPort<bool>("qr_nav")};
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<bool> msg = getInput<bool>("qr_nav");
        if (msg == false) {
            try {
                // Start the Python node
                std::string command = "ros2 run navigation Qr_navigation_node";
                int result = std::system(command.c_str());

                if (result != 0) {
                    throw std::runtime_error("Failed to start QR NAVIGATION node.");
                }
                BT::TreeNode::setOutput("qr_nav", true);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("GoToQr"), "Exception: %s", e.what());
                return BT::NodeStatus::FAILURE;
            }
        }

        // You can add additional logic here to check the status of the node if needed
        return BT::NodeStatus::SUCCESS;
    }

};

class GripperCancel : public BT::SyncActionNode
{
public:
    explicit GripperCancel(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully opened gripper"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};


int main()
{
    BT::BehaviorTreeFactory factory;

    //can subtree

    BT::PortsList found_can_port = {BT::InputPort<bool>("canfound")};
    factory.registerSimpleCondition("FoundCan",foundCan,found_can_port);

    factory.registerNodeType<SearchCan>("SearchCan");

    factory.registerSimpleCondition("IsAtCan", std::bind(isAtCan));

    factory.registerNodeType<GoToCan>("GoToCan");

    factory.registerNodeType<Gripper>("Gripper");

    // QR subtree

    BT::PortsList found_qr_port = {BT::InputPort<bool>("QRfound")};
    factory.registerSimpleCondition("FoundQR", foundQr, found_qr_port);

    factory.registerNodeType<SearchQr>("SearchQR");  

    factory.registerSimpleCondition("IsAtQR", std::bind(isAtQr));

    factory.registerNodeType<GoToQr>("GoToQR"); 

    factory.registerNodeType<GripperCancel>("GripperCancel");     

    //create Tree
    auto tree = factory.createTreeFromFile("./../behavior_tree.xml");

    //Blackboard
    tree.rootBlackboard()->set<bool>("canfound",false);
    tree.rootBlackboard()->set<bool>("process_started",false);
    tree.rootBlackboard()->set<bool>("can_nav",false);
    tree.rootBlackboard()->set<bool>("QRfound",false);
    tree.rootBlackboard()->set<bool>("qr_process_started",false);
    tree.rootBlackboard()->set<bool>("qr_nav",false);
    
    //execute the tree
    //while(true){
        tree.tickRoot();
    //}
    
    
    

    return 0;
}