#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <crs_msgs/srv/run_robot_script.hpp>

static const std::string CONTROLLER_CHANGER_SERVICE = "compliance_controller_on";
static const std::string TOGGLE_SANDER_SERVICE = "toggle_sander";
static const std::string RECLAIM_REMOTE_CONTROL_SERVICE = "reconnect_to_robot";
static const std::string CRS_RUN_ROBOT_SCRIPT_SERVICE = "run_robot_script";

class URRobotCommsSim : public rclcpp::Node
{
public:
  URRobotCommsSim() : Node("ur_robot_comms_sim")
  {
    // ROS communications
    controller_changer_service_ = this->create_service<std_srvs::srv::SetBool>(
        CONTROLLER_CHANGER_SERVICE,
        std::bind(&URRobotCommsSim::simControllerChangeCB, this, std::placeholders::_1, std::placeholders::_2));
    ur_io_service_ = this->create_service<std_srvs::srv::SetBool>(
        TOGGLE_SANDER_SERVICE,
        std::bind(&URRobotCommsSim::simUrIoCB, this, std::placeholders::_1, std::placeholders::_2));
    reconnect_robot_service_ = this->create_service<std_srvs::srv::Trigger>(
        RECLAIM_REMOTE_CONTROL_SERVICE,
        std::bind(&URRobotCommsSim::reconnectCB, this, std::placeholders::_1, std::placeholders::_2));
    load_robot_script_service_ = this->create_service<crs_msgs::srv::RunRobotScript>(
        CRS_RUN_ROBOT_SCRIPT_SERVICE,
        std::bind(&URRobotCommsSim::runRobotScriptCB, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void simControllerChangeCB(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "SIM CONTROLLER CHANGE");
    response->success = true;
  }
  void simUrIoCB(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "SIM IO CHANGE");
    response->success = true;
  }
  void reconnectCB(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "SIM RECONNECT");
    response->success = true;
  }
  void runRobotScriptCB(std::shared_ptr<crs_msgs::srv::RunRobotScript::Request> request,
                        std::shared_ptr<crs_msgs::srv::RunRobotScript::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "SIM RUN ROBOT SCRIPT");
    response->success = true;
  }
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr controller_changer_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ur_io_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_robot_service_;
  rclcpp::Service<crs_msgs::srv::RunRobotScript>::SharedPtr load_robot_script_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<URRobotCommsSim>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
