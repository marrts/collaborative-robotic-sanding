#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//#include <crs_motion_planning/path_processing_utils.h>
#include <crs_motion_planning/path_planning_utils.h>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

static const std::vector<double> COEFFICIENTS {10, 10, 10, 10, 10, 10};



class SurfaceServer: public rclcpp::Node
{
public:
  SurfaceServer()
    : Node("surface_server_node"),
      clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_buffer_(clock_),
      tf_listener_(tf_buffer_)
  {
    // ROS parameters
    this->declare_parameter("urdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/crs_v3.urdf");
    this->declare_parameter("srdf_path", ament_index_cpp::get_package_share_directory("crs_support") + "/urdf/ur10e_robot.srdf");
    this->declare_parameter("base_link_frame", "world");
    this->declare_parameter("manipulator_group", "manipulator");

    // ROS communications
    load_paths_service_ = this->create_service<std_srvs::srv::Trigger>("load_paths", std::bind(&SurfaceServer::planService, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",1);
    original_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("original_raster_paths",1);
    corrected_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fixed_raster_paths",1);
    failed_vertex_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("failed_vertices",1);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&SurfaceServer::jointCallback, this, std::placeholders::_1));

    std::string urdf_path, srdf_path;
    urdf_path = this->get_parameter("urdf_path").as_string();
    srdf_path = this->get_parameter("srdf_path").as_string();
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(urdf_path);
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(srdf_path);
    srdf_xml_string << srdf_in.rdbuf();

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);

    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    manipulator_ = this->get_parameter("manipulator_group").as_string();
    toolpath_filepath_ = ament_index_cpp::get_package_share_directory("crs_support") + "/toolpaths/scanned_part1/job_90degrees.yaml";
  }
private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    curr_joint_state_ = *joint_msg;
  }
  void planService(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {

      // Load rasters and get them in usable form
      std::string waypoint_origin_frame = "part";
      std::vector<geometry_msgs::msg::PoseArray> raster_strips;
      crs_motion_planning::parsePathFromFile(toolpath_filepath_, waypoint_origin_frame, raster_strips);
      geometry_msgs::msg::PoseArray strip_of_interset;
      for (auto strip : raster_strips)
      {
          strip_of_interset.poses.insert(strip_of_interset.poses.end(), strip.poses.begin(), strip.poses.end());
      }


      // Get transform between world and part
      tf2::TimePoint time_point = tf2::TimePointZero;
      geometry_msgs::msg::TransformStamped world_to_goal_frame;
      try
      {
        world_to_goal_frame = tf_buffer_.lookupTransform("world", "part", time_point);
      }
      catch (tf2::LookupException &e)
      {
        response->success = false;
        response->message = "TF lookup failed: " + std::string(e.what());
        return;
      }

      std::vector<geometry_msgs::msg::PoseArray> raster_strips_world_frame;
      for (auto strip : raster_strips)
      {
          geometry_msgs::msg::PoseArray curr_strip, ar_strip;
          for (size_t i = 0; i < strip.poses.size(); ++i)
          {
              geometry_msgs::msg::PoseStamped surface_pose_world_frame, surface_pose_og_frame;
              surface_pose_og_frame.pose = strip.poses[i];
              surface_pose_og_frame.header = strip.header;
              tf2::doTransform(surface_pose_og_frame, surface_pose_world_frame, world_to_goal_frame);
              geometry_msgs::msg::Pose sf_pose_wf = surface_pose_world_frame.pose;
              curr_strip.poses.push_back(std::move(sf_pose_wf));
          }
//          crs_motion_planning::addApproachAndRetreat(curr_strip, 0.05, 0.05, ar_strip);
//          raster_strips_world_frame.push_back(ar_strip);
          raster_strips_world_frame.push_back(curr_strip);
      }
      // Display rasters on part
      visualization_msgs::msg::MarkerArray mark_array_msg;
      crs_motion_planning::rasterStripsToMarkerArray(raster_strips_world_frame, "world", mark_array_msg, {1.0, 0.0, 0.0, 1.0}, -0.01);
      original_path_publisher_->publish(mark_array_msg);

      // TODO: add approach and retreat to each raster

      // Set up planning config variable
      crs_motion_planning::descartesConfig descartes_config;
      descartes_config.axial_step = 0.1;
      descartes_config.collision_safety_margin = 0.0075;

      crs_motion_planning::trajoptSurfaceConfig trajopt_surface_config;
      trajopt_surface_config.smooth_velocities = false;
      trajopt_surface_config.smooth_accelerations = false;
      trajopt_surface_config.smooth_jerks = false;
      tesseract_motion_planners::CollisionCostConfig coll_cost_config_srfc, coll_cost_config_fs;
      coll_cost_config_srfc.enabled = false;
      trajopt_surface_config.coll_cst_cfg = coll_cost_config_srfc;
      tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_srfc, coll_cnt_config_fs;
      coll_cnt_config_srfc.enabled = true;
      coll_cnt_config_srfc.safety_margin = 0.001;
      trajopt_surface_config.coll_cnt_cfg = coll_cnt_config_srfc;
      Eigen::VectorXd surface_coeffs(6);
      surface_coeffs << 10, 10, 10, 10, 10, 0;
      trajopt_surface_config.surface_coeffs = surface_coeffs;
      trajopt_surface_config.waypoints_critical = false;

      crs_motion_planning::omplConfig ompl_config;
      ompl_config.collision_safety_margin = 0.02;
      ompl_config.planning_time = 10;
      ompl_config.simplify = false;

      crs_motion_planning::trajoptFreespaceConfig trajopt_freespace_config;
      coll_cost_config_fs = coll_cost_config_srfc;
      coll_cost_config_fs.enabled = true;
      coll_cost_config_fs.buffer_margin = 0.025;
      coll_cnt_config_fs = coll_cnt_config_srfc;
      coll_cnt_config_fs.safety_margin = 0.01;
      trajopt_freespace_config.coll_cst_cfg = coll_cost_config_fs;
      trajopt_freespace_config.coll_cnt_cfg = coll_cnt_config_fs;
      trajopt_freespace_config.longest_valid_segment_fraction = 0.005;

      Eigen::Isometry3d tool_offset;
      tool_offset.setIdentity();
      Eigen::VectorXd start_pose(6), end_pose(6);
      start_pose << 0, 0, 0, 0, 0, 0;
      end_pose = start_pose;

      auto path_plan_config = std::make_shared<crs_motion_planning::pathPlanningConfig>();
      path_plan_config->tesseract_local = tesseract_local_;
      path_plan_config->descartes_config = descartes_config;
      path_plan_config->trajopt_surface_config = trajopt_surface_config;
      path_plan_config->ompl_config = ompl_config;
      path_plan_config->trajopt_freespace_config = trajopt_freespace_config;
      path_plan_config->manipulator = manipulator_;
      path_plan_config->world_frame = "world";
      path_plan_config->robot_base_frame = "base_link";
      path_plan_config->tool0_frame = "tool0";
      path_plan_config->tcp_frame = "sander_center_link";
      path_plan_config->rasters = raster_strips_world_frame;
      path_plan_config->smooth_velocities = false;
      path_plan_config->smooth_accelerations = false;
      path_plan_config->smooth_jerks = false;
      path_plan_config->max_joint_vel = 5.0;//1.5;
      path_plan_config->tool_speed = 0.4;//0.3;
      path_plan_config->tool_offset = tool_offset;
      path_plan_config->minimum_raster_length = 4;
      path_plan_config->add_approach_and_retreat = true;
      path_plan_config->approach_distance = 0.05;
      path_plan_config->retreat_distance = 0.05;
      path_plan_config->use_start = true;
      path_plan_config->start_pose = start_pose;
      path_plan_config->use_end = true;
      path_plan_config->end_pose = end_pose;

      // Create crsMotionPlanner class
      crs_motion_planning::crsMotionPlanner crs_motion_planner(path_plan_config);

      auto path_plan_results = std::make_shared<crs_motion_planning::pathPlanningResults>();
      bool success = crs_motion_planner.generateProcessPlan(path_plan_results);

      std::vector<trajectory_msgs::msg::JointTrajectory> trajopt_trajectories;
//      trajopt_trajectories = path_plan_results->final_raster_trajectories;
      trajopt_trajectories = path_plan_results->final_trajectories;

      if (success || trajopt_trajectories.size() > 0)
      {
          visualization_msgs::msg::MarkerArray temp_mark_array_msg, pub_mark_array_msg;
          crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->solved_rasters, path_plan_config->world_frame, temp_mark_array_msg, {1.0, 0.0, 1.0, 0.0}, -0.025);
          crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->failed_rasters, path_plan_config->world_frame, temp_mark_array_msg, {1.0, 1.0, 0.0, 0.0}, -0.025);
          crs_motion_planning::rasterStripsToMarkerArray(path_plan_results->skipped_rasters, path_plan_config->world_frame, temp_mark_array_msg, {1.0, 1.0, 1.0, 0.0}, -0.025);
          pub_mark_array_msg.markers.insert(pub_mark_array_msg.markers.end(), temp_mark_array_msg.markers.begin(), temp_mark_array_msg.markers.end());
          corrected_path_publisher_->publish(pub_mark_array_msg);

          visualization_msgs::msg::Marker failed_vertex_markers;
          crs_motion_planning::failedEdgesToMarkerArray(path_plan_results->unreachable_waypoints, path_plan_config->world_frame, failed_vertex_markers, {1.0, 0.0, 1.0, 1.0}, 0.01);
          failed_vertex_publisher_->publish(failed_vertex_markers);

          for (size_t i = 0; i < trajopt_trajectories.size(); ++i)
          {
              std::cout << "PUBLISHING TRAJECTORY " << i+1 << " OF " << trajopt_trajectories.size() << std::endl;
              traj_publisher_->publish(trajopt_trajectories[i]);
              if (i == 0)
              {
                  std::this_thread::sleep_for(std::chrono::seconds(3));
              }
              std::this_thread::sleep_for(std::chrono::seconds(5));
          }
          std::cout << "ALL DONE" << std::endl;
          response->success = true;
          response->message = "TRAJECTORIES PUBLISHED";
      }
      else
      {
          response->success = false;
          response->message = "Failed to generate preplan";
      }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_paths_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_path_publisher_, corrected_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_vertex_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  tesseract::Tesseract::Ptr tesseract_local_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::JointState curr_joint_state_;

  std::string base_link_frame_;
  std::string manipulator_;
  std::string toolpath_filepath_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceServer>());
  rclcpp::shutdown();
  return 0;
}
