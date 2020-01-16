#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

//#include <crs_msgs/srv/get_joint_traj.hpp>

#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
//#include <tesseract_motion_planners/trajopt/config/utils.h>
//#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
//#include <tesseract_motion_planners/core/waypoint.h>
//#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
//#include <tesseract_motion_planners/ompl/conversions.h>
//#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
//#include <tesseract_motion_planners/ompl/ompl_settings.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>

#include <trajopt/problem_description.hpp>

//#include <tesseract_kinematics/core/forward_kinematics.h>

#include <tesseract_rosutils/utils.h>
//#include <trajopt/plot_callback.hpp>
//#include <trajopt_utils/logging.hpp>

//#include <descartes_light/descartes_light.h>
//#include <descartes_tesseract/descartes_tesseract_collision_checker.h>
//#include <descartes_samplers/evaluators/distance_edge_evaluator.h>
//#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
//#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

//#include <Eigen/Geometry>

void tesseract_rosutils_toMsg(trajectory_msgs::msg::JointTrajectory& traj_msg,
                              const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(joint_names.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));

  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint jtp;
    jtp.positions.resize(static_cast<size_t>(traj.cols()));

    for (int j = 0; j < traj.cols(); ++j)
    {
      if (i == 0)
        traj_msg.joint_names[static_cast<size_t>(j)] = joint_names[static_cast<size_t>(j)];

      jtp.positions[static_cast<size_t>(j)] = traj(i, j);
    }

    jtp.time_from_start = rclcpp::Duration(i, 0);  // TODO: re-enable with rclcpp::Duration
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

class PlanningTest: public rclcpp::Node
{
public:
  PlanningTest()
    : Node("planning_test_node"),
      clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_buffer_(clock_),
      tf_listener_(tf_buffer_)
  {
//    plan_service_ = this->create_service<crs_msgs::srv::GetJointTraj>("test_plan", std::bind(&PlanningTest::plan_service, this, std::placeholders::_1, std::placeholders::_2));
    plan_service_ = this->create_service<std_srvs::srv::Trigger>("test_plan", std::bind(&PlanningTest::plan_service, this, std::placeholders::_1, std::placeholders::_2));
    plan_service2_ = this->create_service<std_srvs::srv::Trigger>("test_plan2", std::bind(&PlanningTest::plan_service2, this, std::placeholders::_1, std::placeholders::_2));
    plan_service3_ = this->create_service<std_srvs::srv::Trigger>("test_plan3", std::bind(&PlanningTest::plan_service3, this, std::placeholders::_1, std::placeholders::_2));
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_trajectory_test",10);
    joint_state_listener_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 2, std::bind(&PlanningTest::joint_callback, this, std::placeholders::_1));

    joint_name_list_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    std::string urdf_path, srdf_path;
    urdf_path = "/home/tmarr/ros2_workspaces/collaborative-robotic-sanding/install/crs_support/share/crs_support/urdf/crs.urdf";
    srdf_path = "/home/tmarr/ros2_workspaces/collaborative-robotic-sanding/install/crs_support/share/crs_support/urdf/ur10e_robot.srdf";
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(urdf_path);
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(srdf_path);
    srdf_xml_string << srdf_in.rdbuf();

    tesseract_local_ = std::make_shared<tesseract::Tesseract>();
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);

    std::unordered_map<std::string, double> ipos;
    ipos["shoulder_pan_joint"] = 0.0;
    ipos["shoulder_lift_joint"] = 0.0;
    ipos["elbow_joint"] = 0.0;
    ipos["wrist_1_joint"] = 0.0;
    ipos["wrist_2_joint"] = 0.0;
    ipos["wrist_3_joint"] = 0.0;
    tesseract_local_->getEnvironment()->setState(ipos);

    std::cout << "DONE" << std::endl;
  }
private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    curr_joint_state_ = *joint_msg;
  }
  void plan_service(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
//  void plan_service(std::shared_ptr<crs_msgs::srv::GetJointTraj::Request> request,
//                    std::shared_ptr<crs_msgs::srv::GetJointTraj::Response> response)
  {
    tesseract_motion_planners::TrajOptPlannerConfig::Ptr traj_config;
    trajopt::TrajOptProb::Ptr traj_problem;

//    tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_local_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    trajopt::ProblemConstructionInfo pci(tesseract_local_);
    int steps = 10;

    // Populate bsic info
    pci.basic_info.n_steps = steps;
    pci.basic_info.manip = "manipulator";
    pci.basic_info.start_fixed = false;
    pci.basic_info.use_time = false;

    // Create kinematic object
    pci.kin = pci.getManipulator(pci.basic_info.manip);

    // Populate init info
    tesseract_environment::EnvState::ConstPtr current_state = pci.env->getCurrentState();
    Eigen::VectorXd start_pos;
    start_pos.resize(pci.kin->numJoints());
    int cnt = 0;
    for (const auto& j : pci.kin->getJointNames())
    {
      start_pos[cnt] = current_state->joints.at(j);
      ++cnt;
    }

    pci.init_info.type = trajopt::InitInfo::STATIONARY;
    pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

    // Populate Cost Info
    std::shared_ptr<trajopt::JointVelTermInfo> jv = std::shared_ptr<trajopt::JointVelTermInfo>(new trajopt::JointVelTermInfo);
    jv->coeffs = std::vector<double>(6, 5.0);
    jv->targets = std::vector<double>(6, 0.0);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_vel";
    jv->term_type = trajopt::TT_COST;
    pci.cost_infos.push_back(jv);

    std::shared_ptr<trajopt::CollisionTermInfo> collision = std::shared_ptr<trajopt::CollisionTermInfo>(new trajopt::CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
//    collision->continuous = false;
    collision->first_step = 0;
    collision->last_step = pci.basic_info.n_steps - 1;
//    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.035, 20);

    // Populate Constraints
    double delta = 0.125 / pci.basic_info.n_steps;
    for (auto i = 0; i < pci.basic_info.n_steps; ++i)
    {
      std::shared_ptr<trajopt::CartPoseTermInfo> pose = std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
      pose->term_type = trajopt::TT_CNT;
      pose->name = "waypoint_cart_" + std::to_string(i);
      pose->link = "tool0";
      pose->timestep = i;
      pose->xyz = Eigen::Vector3d(0.0, 0.0 + delta * i, 1.25);
      pose->wxyz = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
      pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
      pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
      pci.cnt_infos.push_back(pose);
    }

    traj_problem = trajopt::ConstructProblem(pci);

    std::vector<tesseract_collision::ContactResultMap> collisions;
    tesseract_collision::ContinuousContactManager::Ptr manager = traj_problem->GetEnv()->getContinuousContactManager();
    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(traj_problem->GetEnv()->getSceneGraph(),
                                                                                                                   traj_problem->GetKin()->getActiveLinkNames(),
                                                                                                                   traj_problem->GetEnv()->getCurrentState()->transforms);

    manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
    manager->setContactDistanceThreshold(0);
    collisions.clear();

    bool found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_problem->GetEnv()->getStateSolver(), traj_problem->GetKin()->getJointNames(), traj_problem->GetInitTraj());
    std::cout << ((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free")) << std::endl;

    sco::BasicTrustRegionSQP opt(traj_problem);

    opt.initialize(trajopt::trajToDblVec(traj_problem->GetInitTraj()));
    opt.optimize();

    collisions.clear();
    found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_problem->GetEnv()->getStateSolver(), traj_problem->GetKin()->getJointNames(), trajopt::getTraj(opt.x(), traj_problem->GetVars()));

    std::cout << ((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free")) << std::endl;

    tesseract_common::TrajArray traj_array = trajopt::getTraj(opt.x(), traj_problem->GetVars());

    Eigen::MatrixXd cumulative_trajectory(traj_array.rows(), traj_array.cols());

    cumulative_trajectory << traj_array;

    trajectory_msgs::msg::JointTrajectory cumulative_joint_trajectory;
    tesseract_rosutils_toMsg(cumulative_joint_trajectory, joint_name_list_, cumulative_trajectory);
    cumulative_joint_trajectory.header.frame_id = "world";
    traj_publisher_->publish(cumulative_joint_trajectory);

//    response->output_trajectory = cumulative_joint_trajectory;
    response->success = true;
    response->message = "It worked";
  }

  void plan_service2(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string target_frame = "camera_link_optical";
    geometry_msgs::msg::TransformStamped base_to_target_frame;
    tf2::TimePoint curr_time = tf2_ros::fromMsg(curr_joint_state_.header.stamp);
    try
    {
      base_to_target_frame = tf_buffer_.lookupTransform("world", target_frame, curr_time, tf2::Duration(std::chrono::seconds(3)));
    }
    catch (tf2::LookupException &e)
    {
      response->message = "Failed to perform tf lookup";
      response->success = false;
      return;
    }
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci_pointer;
    trajopt::TrajOptProb::Ptr traj_problem;
    int steps = 5;

    Eigen::Isometry3d tcp_eigen;
    tcp_eigen.setIdentity();

    tesseract_motion_planners::TrajOptPlannerDefaultConfig traj_pc(tesseract_local_, "manipulator", target_frame, tcp_eigen);

    // Initialize vector of target waypoints
    std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;

    // Establish initial waypoint
    Eigen::Vector3d init_pose(base_to_target_frame.transform.translation.x,base_to_target_frame.transform.translation.y,base_to_target_frame.transform.translation.z);
    Eigen::Quaterniond init_ori(base_to_target_frame.transform.rotation.w,base_to_target_frame.transform.rotation.x,base_to_target_frame.transform.rotation.y,base_to_target_frame.transform.rotation.z);
    tesseract_motion_planners::CartesianWaypoint::Ptr init_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(init_pose, init_ori);
    init_waypoint->setIsCritical(true);
    trgt_wypts.push_back(init_waypoint);

     // Iterate through target waypoints
    for (auto i = 0; i < steps; ++i)
    {
      tesseract_motion_planners::CartesianWaypoint::Ptr curr_waypoint_cart = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(Eigen::Vector3d(-0.5, -0.25 + 0.125 * i, 1.5), Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));
      curr_waypoint_cart->setIsCritical(true);
      Eigen::VectorXd coeffs(6);
      coeffs << 10, 10, 10, 0, 0, 0;
      curr_waypoint_cart->setCoefficients(coeffs);
      trgt_wypts.push_back(curr_waypoint_cart);
    }

    // Write target waypoints to trajopt planning config
    traj_pc.target_waypoints = trgt_wypts;

    // Set longest valid segment to check
    traj_pc.longest_valid_segment_length = 0.01;

    // Generate trajopt problem
    traj_pc.generate();

    traj_problem = traj_pc.prob;

    // Setup tesseract collision environment
    std::vector<tesseract_collision::ContactResultMap> collisions;
    tesseract_collision::ContinuousContactManager::Ptr manager = traj_problem->GetEnv()->getContinuousContactManager();
    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(traj_problem->GetEnv()->getSceneGraph(),
                                                                                                                   traj_problem->GetKin()->getActiveLinkNames(),
                                                                                                                   traj_problem->GetEnv()->getCurrentState()->transforms);

    manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
    manager->setContactDistanceThreshold(0);
    collisions.clear();

    // Check for collisions in initial trajectory
    bool found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_problem->GetEnv()->getStateSolver(), traj_problem->GetKin()->getJointNames(), traj_problem->GetInitTraj());
    std::cout << ((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free")) << std::endl;

    // Setup and run optimization problem
    sco::BasicTrustRegionSQP opt(traj_problem);
    opt.initialize(trajopt::trajToDblVec(traj_problem->GetInitTraj()));
    opt.optimize();

    // Check for collisions in final trajectory
    collisions.clear();
    found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_problem->GetEnv()->getStateSolver(), traj_problem->GetKin()->getJointNames(), trajopt::getTraj(opt.x(), traj_problem->GetVars()));
    std::cout << ((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free")) << std::endl;

    // Store generated trajectory
    tesseract_common::TrajArray traj_array = trajopt::getTraj(opt.x(), traj_problem->GetVars());
    Eigen::MatrixXd cumulative_trajectory(traj_array.rows(), traj_array.cols());
    cumulative_trajectory << traj_array;

    // Convert trajectory to ROSmsg
    trajectory_msgs::msg::JointTrajectory cumulative_joint_trajectory;
    tesseract_rosutils_toMsg(cumulative_joint_trajectory, joint_name_list_, cumulative_trajectory);
    cumulative_joint_trajectory.header.frame_id = "world";
    // Publish trajectory
    traj_publisher_->publish(cumulative_joint_trajectory);

    response->success = true;
    response->message = "It worked";
  }
  void plan_service3(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string target_frame = "camera_link_optical";
    geometry_msgs::msg::TransformStamped base_to_target_frame;
    tf2::TimePoint curr_time = tf2_ros::fromMsg(curr_joint_state_.header.stamp);
    try
    {
      base_to_target_frame = tf_buffer_.lookupTransform("world", target_frame, curr_time, tf2::Duration(std::chrono::seconds(3)));
    }
    catch (tf2::LookupException &e)
    {
      response->message = "Failed to perform tf lookup";
      response->success = false;
      return;
    }
    int num_steps = 30;
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci_pointer;
    trajopt::TrajOptProb::Ptr traj_problem;
    tesseract_motion_planners::TrajOptMotionPlanner traj_motion_planner;

    Eigen::Isometry3d tcp_eigen;
    tcp_eigen.setIdentity();

    auto traj_pc = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(tesseract_local_, "manipulator", target_frame, tcp_eigen);

    // Initialize vector of target waypoints
    std::vector<tesseract_motion_planners::Waypoint::Ptr> trgt_wypts;


    // Establish initial waypoint
    Eigen::Vector3d init_pose(base_to_target_frame.transform.translation.x,base_to_target_frame.transform.translation.y,base_to_target_frame.transform.translation.z);
    Eigen::Quaterniond init_ori(base_to_target_frame.transform.rotation.w,base_to_target_frame.transform.rotation.x,base_to_target_frame.transform.rotation.y,base_to_target_frame.transform.rotation.z);
    tesseract_motion_planners::CartesianWaypoint::Ptr init_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(init_pose, init_ori);
    init_waypoint->setIsCritical(false);

    Eigen::Vector3d goal_pose(-0.5, 1.0, 1.5);
    Eigen::Quaterniond goal_ori(0.0, 0.0, 1.0, 0.0);
    tesseract_motion_planners::CartesianWaypoint::Ptr goal_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(goal_pose, goal_ori);
    goal_waypoint->setIsCritical(false);
    Eigen::VectorXd coeffs(6);
    coeffs << 10, 10, 10, 0, 0, 0;
    goal_waypoint->setCoefficients(coeffs);
    init_waypoint->setCoefficients(coeffs);

    // Add cart waypoints
//    trgt_wypts.push_back(init_waypoint);
//    trgt_wypts.push_back(goal_waypoint);

    // Define joint waypoints
    Eigen::VectorXd init_joint_pose(6), goal_joint_pose(6);
//    init_joint_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    init_joint_pose << curr_joint_state_.position[0], curr_joint_state_.position[1], curr_joint_state_.position[2], curr_joint_state_.position[3], curr_joint_state_.position[4], curr_joint_state_.position[5];
    tesseract_motion_planners::JointWaypoint::Ptr joint_init_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(init_joint_pose, joint_name_list_);
//    goal_joint_pose << 1.57, -1.57, 0.0, 0.0, -1.57, 0.0;
    goal_joint_pose << 6.28, 0.0, 0.0, 0.0, 0.0, 0.0;
    tesseract_motion_planners::JointWaypoint::Ptr joint_goal_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(goal_joint_pose, joint_name_list_);
    joint_init_waypoint->setIsCritical(false);
    joint_goal_waypoint->setIsCritical(false);
    Eigen::VectorXd j_coeffs(6);
    j_coeffs << 10, 10, 10, 10, 10, 10;
    joint_goal_waypoint->setCoefficients(j_coeffs);

    // Add waypoints
    trgt_wypts.push_back(joint_init_waypoint);
//    trgt_wypts.push_back(joint_goal_waypoint);
    trgt_wypts.push_back(goal_waypoint);

    auto inv_kin_mngr = tesseract_local_->getInvKinematicsManager();
    Eigen::VectorXd sols;
    Eigen::Isometry3d goal_pose_iso;
    goal_pose_iso.translation() = goal_pose;
    goal_pose_iso.linear() = goal_ori.matrix();
    inv_kin_mngr->getInvKinematicSolver("manipulator")->calcInvKin(sols, goal_pose_iso, init_joint_pose, target_frame);
    std::cout << "INV KIN SOL: " << sols << std::endl;

    // Write target waypoints to trajopt planning config
    traj_pc->target_waypoints = trgt_wypts;

    traj_pc->num_steps = num_steps;
//    traj_pc->init_type = trajopt::InitInfo::GIVEN_TRAJ;
//    traj_pc->init_type = trajopt::InitInfo::JOINT_INTERPOLATED;
    trajopt::TrajArray seed_traj;
    seed_traj.resize(num_steps, goal_joint_pose.rows());

    // Perform Joint interpolation
    tesseract_environment::EnvState state(*(tesseract_local_->getEnvironmentConst()->getCurrentState()));
    Eigen::VectorXd start_pos(joint_name_list_.size());
    int i = 0;
    for (const auto& joint : joint_name_list_)
    {
      assert(state.joints.find(joint) != state.joints.end());
      start_pos[i] = state.joints[joint];
      ++i;
    }

    seed_traj.resize(num_steps, goal_joint_pose.rows());
    for (int idof = 0; idof < start_pos.rows(); ++idof)
    {
      seed_traj.col(idof) = Eigen::VectorXd::LinSpaced(num_steps, start_pos(idof), goal_joint_pose(idof));
    }
//    traj_pc->seed_trajectory = seed_traj;
    traj_pc->smooth_jerks = true;
    traj_pc->smooth_velocities = true;
    traj_pc->smooth_accelerations = true;
    traj_pc->collision_check = true;
    traj_pc->collision_continuous = true;
    traj_pc->collision_coeff = 20;
//    traj_pc->params.cnt_tolerance = 0.02;


    // Set longest valid segment to check
    traj_pc->longest_valid_segment_length = 0.05;
    traj_motion_planner.setConfiguration(traj_pc);
//    traj_motion_planner.
    tesseract_motion_planners::PlannerResponse plan_resp;
    traj_motion_planner.solve(plan_resp, true);

    // Setup tesseract collision environment
    std::vector<tesseract_collision::ContactResultMap> collisions;
    tesseract_collision::ContinuousContactManager::Ptr manager = traj_pc->tesseract->getEnvironmentConst()->getContinuousContactManager();
    collisions.clear();

    // Check for collisions in initial trajectory
    bool found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_pc->tesseract->getEnvironmentConst()->getStateSolver(), joint_name_list_, seed_traj);
    std::cout << ((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free")) << std::endl;

    // Check for collisions in final trajectory
    collisions.clear();
    found = tesseract_environment::checkTrajectory(collisions, *manager, *traj_pc->tesseract->getEnvironmentConst()->getStateSolver(), joint_name_list_, plan_resp.joint_trajectory.trajectory);
    std::cout << ((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free")) << std::endl;

    // Store generated trajectory
    tesseract_common::TrajArray traj_array = plan_resp.joint_trajectory.trajectory;//trajopt::getTraj(opt.x(), traj_problem->GetVars());
    Eigen::MatrixXd cumulative_trajectory(traj_array.rows(), traj_array.cols());
    cumulative_trajectory << traj_array;

    // Convert trajectory to ROSmsg
    trajectory_msgs::msg::JointTrajectory cumulative_joint_trajectory;
    tesseract_rosutils_toMsg(cumulative_joint_trajectory, joint_name_list_, cumulative_trajectory);
    cumulative_joint_trajectory.header.frame_id = "world";

    // Modify time
    builtin_interfaces::msg::Time joint_time = cumulative_joint_trajectory.header.stamp;
    int time_to_complete_traj = 3;
    for (int i = 0; i < num_steps; ++i)
    {
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.sec = 0;
      cumulative_joint_trajectory.points[static_cast<size_t>(i)].time_from_start.nanosec = 2e8;
    }
    // Publish trajectory
    traj_publisher_->publish(cumulative_joint_trajectory);

    response->success = true;
    response->message = "It worked";
  }

//  rclcpp::Service<crs_msgs::srv::GetJointTraj>::SharedPtr plan_service_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service2_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service3_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_listener_;
  tesseract::Tesseract::Ptr tesseract_local_;

  std::vector<std::string> joint_name_list_;
  sensor_msgs::msg::JointState curr_joint_state_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningTest>());
  rclcpp::shutdown();
  return 0;
}
