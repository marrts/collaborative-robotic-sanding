#include <crs_motion_planning/path_planning_utils.h>

bool crs_motion_planning::generateDescartesSeed(const tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                                                const std::shared_ptr<const tesseract_environment::Environment> env,
                                                const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                                                const descartes_light::KinematicsInterfaceD::Ptr &kin_interface,
                                                const double &axial_step,
                                                const bool &allow_collisions,
                                                const double &collision_safety_margin,
                                                std::vector<std::size_t>& failed_edges,
                                                std::vector<std::size_t>& failed_vertices,
                                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    tesseract_common::TransformMap curr_transforms = env->getCurrentState()->transforms;

    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(), kin->getActiveLinkNames(), curr_transforms);

    auto collision_checker = std::make_shared<tesseract_motion_planners::DescartesCollisionD>(env, adjacency_map->getActiveLinkNames(), kin->getJointNames(), collision_safety_margin);

    std::vector<descartes_light::PositionSamplerD::Ptr> sampler_result;

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        Eigen::Isometry3d current_waypoint_pose;
        tf2::fromMsg(waypoints[i].pose, current_waypoint_pose);
        sampler_result.emplace_back(std::make_shared<descartes_light::AxialSymmetricSamplerD>(current_waypoint_pose,
                                                                                                     kin_interface,
                                                                                                     axial_step,
                                                                                                     std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
                                                                                                     allow_collisions));
    }

    auto edge_eval = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluatorD>(kin_interface->dof());
    auto timing_constraint = std::vector<descartes_core::TimingConstraintD>(sampler_result.size(), std::numeric_limits<double>::max());

    descartes_light::SolverD graph_builder(kin_interface->dof());

    if (!graph_builder.build(std::move(sampler_result),
                             std::move(timing_constraint),
                             std::move(edge_eval)))
    {
        failed_edges = graph_builder.getFailedEdges();
        failed_vertices = graph_builder.getFailedVertices();
        return false;
    }

    std::vector<double> solution;
    if (!graph_builder.search(solution))
    {
      return false;
    }

    Eigen::Map<Eigen::VectorXd> solution_vec (&solution[0], solution.size());
    Eigen::VectorXd seed_traj(solution_vec.size());
    seed_traj << solution_vec;

    int n_rows = seed_traj.size() / kin_interface->dof();
    Eigen::MatrixXd joint_traj_eigen_out = Eigen::Map<Eigen::MatrixXd>(seed_traj.data(), kin_interface->dof(), n_rows).transpose();

    crs_motion_planning::tesseractRosutilsToMsg(joint_trajectory, kin->getJointNames(), joint_traj_eigen_out);
    return true;
}