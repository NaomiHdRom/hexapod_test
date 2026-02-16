#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "leg1");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);



    // Named goal

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("leg1_HOME");

     moveit::planning_interface::MoveGroupInterface::Plan plan1;
     bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

     if (success1){
         arm.execute(plan1);
     }



    rclcpp::shutdown();
    spinner.join();
    return 0;
}