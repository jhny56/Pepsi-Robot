#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;

/*Can subtree*/
BT::NodeStatus foundCan(){
    bool found = false;   
    if (found){
        std::cout << "Can found" << std::endl;
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Can not found" << std::endl;
    return BT::NodeStatus::FAILURE;
}

class SearchCan : public BT::SyncActionNode
{
public:
    explicit SearchCan(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully searched for can"<<std::endl;
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
    explicit GoToCan(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully went to can"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class Gripper : public BT::SyncActionNode
{
public:
    explicit Gripper(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully closed gripper"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

/*QR subtree*/
BT::NodeStatus foundQr(){
    bool foundQR = false;   
    if (foundQR){
        std::cout << "Robot found QR code" << std::endl;
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Robot not found QR code." << std::endl;
    return BT::NodeStatus::FAILURE;
}

class SearchQr : public BT::SyncActionNode
{
public:
    explicit SearchQr(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully searched for QR code"<<std::endl;
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
    explicit GoToQr(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully went to QR code"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus isAtStrip(){
    bool atStrip = false;   
    if (atStrip){
        std::cout << "Robot is at black strip" << std::endl;
        return BT::NodeStatus::SUCCESS;        
    }
    std::cout << "Robot is not at black strip" << std::endl;
    return BT::NodeStatus::FAILURE;
}

class GoToStrip : public BT::SyncActionNode
{
public:
    explicit GoToStrip(const std::string &name) : BT::SyncActionNode(name, {})
    {        
    }    
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::cout<<"Successfully went to black strip"<<std::endl;
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

    factory.registerSimpleCondition("FoundCan", std::bind(foundCan));

    factory.registerNodeType<SearchCan>("SearchCan");

    factory.registerSimpleCondition("IsAtCan", std::bind(isAtCan));

    factory.registerNodeType<GoToCan>("GoToCan");

    factory.registerNodeType<Gripper>("Gripper");

    // QR subtree

    factory.registerSimpleCondition("FoundQR", std::bind(foundQr));

    factory.registerNodeType<SearchQr>("SearchQR");  

    factory.registerSimpleCondition("IsAtQR", std::bind(isAtQr));

    factory.registerNodeType<GoToQr>("GoToQR"); 

    factory.registerSimpleCondition("IsAtStrip", std::bind(isAtStrip));

    factory.registerNodeType<GoToStrip>("GoToStrip"); 

    factory.registerNodeType<GripperCancel>("GripperCancel");     

    //create Tree
    auto tree = factory.createTreeFromFile("./../behavior_tree.xml");

    //execute the tree
    tree.tickRoot();
    
    

    return 0;
}