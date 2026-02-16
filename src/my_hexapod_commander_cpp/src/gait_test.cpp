#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "gait_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // ===== MOVE GROUPS =====
    moveit::planning_interface::MoveGroupInterface fullbody(node, "FULLBODY");
    moveit::planning_interface::MoveGroupInterface gait1(node, "Gait1");

    fullbody.setMaxVelocityScalingFactor(0.6);
    gait1.setMaxVelocityScalingFactor(0.6);

    // ===== 1️⃣ HOME =====
    RCLCPP_INFO(node->get_logger(), "Going to HOME");

    fullbody.setStartStateToCurrentState();
    fullbody.setNamedTarget("HOME");

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (fullbody.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        fullbody.execute(home_plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed HOME");
        return 1;
    }

    // ⏸️ ESPERAR (CLAVE)
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ===== 2️⃣ GAIT1 UP =====
    RCLCPP_INFO(node->get_logger(), "Executing Gait1: lift legs 1-3-5");

    gait1.setStartStateToCurrentState();
    gait1.setNamedTarget("WalkingGait1Up");

    moveit::planning_interface::MoveGroupInterface::Plan gait_plan;
    if (gait1.plan(gait_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        gait1.execute(gait_plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed Gait1");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
