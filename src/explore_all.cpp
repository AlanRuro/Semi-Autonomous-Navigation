#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <stdexcept>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 8);
    std::vector<rclcpp::Node::SharedPtr> nodes;
    std::string nav2_bringup_dir = ament_index_cpp::get_package_share_directory("nav2_bringup");
    std::string slam_toolbox_dir = ament_index_cpp::get_package_share_directory("slam_toolbox");
    std::string nav2_params_file = nav2_bringup_dir + "/params/nav2_params.yaml";
    std::string slam_params_file = slam_toolbox_dir + "/config/mapper_params_online_async.yaml";
    auto nav2_params = rclcpp::ParameterMap();
    auto slam_params = rclcpp::ParameterMap();

    try {
        nav2_params = rclcpp::parameter_map_from_yaml_file(nav2_params_file);
        slam_params = rclcpp::parameter_map_from_yaml_file(slam_params_file);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("autonomous_explore_cpp"),
                     "Failed to load parameter files: %s", e.what());
        return -1;
    }

    auto slam_node = rclcpp::Node::make_shared("slam_toolbox", options);
    slam_node->declare_parameters("", slam_params["slam_toolbox"]);
    nodes.push_back(slam_node);

    auto nav2_node = rclcpp::Node::make_shared("nav2_bringup", options);
    nav2_node->declare_parameters("", nav2_params["/"]);
    nodes.push_back(nav2_node);

    auto explore_node = rclcpp::Node::make_shared("explore", options);
    nodes.push_back(explore_node);

    for (auto &node : nodes) {
        executor->add_node(node);
    }

    auto lifecycle_manager_client = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
        "lifecycle_manager_navigation");
    lifecycle_manager_client->startup();

    while (!lifecycle_manager_client->is_active()) {
        RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"),
                    "Waiting for Navigation2 to become active...");
        std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"), "Navigation2 is active.");

    std::thread spin_thread([&executor]() {
        executor->spin();
    });

    std::this_thread::sleep_for(10s);
    RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"),
                "Updating parameters dynamically...");

    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(explore_node);
    if (parameters_client->wait_for_service(5s)) {
        auto set_parameters_result = parameters_client->set_parameters({
            rclcpp::Parameter("goal_blacklist_radius", 2.0)
        });
        if (set_parameters_result.wait_for(5s) == std::future_status::ready) {
            for (auto &result : set_parameters_result.get()) {
                if (!result.successful) {
                    RCLCPP_ERROR(rclcpp::get_logger("autonomous_explore_cpp"),
                                 "Failed to set parameter: %s", result.reason.c_str());
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"),
                                "Parameter updated successfully.");
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("autonomous_explore_cpp"),
                         "Parameter update timed out.");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("autonomous_explore_cpp"),
                     "Failed to connect to parameter server.");
    }

    RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"), "Exploration started.");
    std::this_thread::sleep_for(5min);

    lifecycle_manager_client->shutdown();
    RCLCPP_INFO(rclcpp::get_logger("autonomous_explore_cpp"), "Shutting down.");

    executor->cancel();
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}