#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using json = nlohmann::json;

std::string read(const std::string& path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read " + path);
  return std::string(std::istreambuf_iterator<char>(input), {});
}

json bounds(const moveit::core::RobotState& state)
{
  json result = json::array();
  for (const auto* joint : state.getRobotModel()->getActiveJointModels())
    for (const auto& name : joint->getVariableNames())
    {
      const auto& b = state.getRobotModel()->getVariableBounds(name);
      const double position = state.getVariablePosition(name);
      if (b.position_bounded_ && (position < b.min_position_ || position > b.max_position_))
        result.push_back({{"joint", name}, {"position", position},
                          {"lower", b.min_position_}, {"upper", b.max_position_}});
    }
  return result;
}

json contacts(planning_scene::PlanningScene& scene, const moveit::core::RobotState& state)
{
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 1000;
  request.max_contacts_per_pair = 1;
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state);
  json pairs = json::array();
  for (const auto& pair : result.contacts)
    pairs.push_back({pair.first.first, pair.first.second});
  return pairs;
}

int main(int argc, char** argv)
{
  if (argc < 5) { std::cerr << "Usage: model_checks MODEL_DIR REPORT_JSON POSE_JSON...\n"; return 2; }
  try
  {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sawyer_moveit_model_checks");
    const std::string directory = argv[1];
    robot_model_loader::RobotModelLoader::Options options(read(directory + "/sawyer.urdf"),
                                                         read(directory + "/sawyer.srdf"));
    options.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoader loader(node, options);
    auto model = loader.getModel();
    if (!model) throw std::runtime_error("MoveIt model failed to load");
    auto group = model->getJointModelGroup("right_arm");
    const std::vector<std::string> expected = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
    if (!group || group->getActiveJointModelNames() != expected)
      throw std::runtime_error("Planning group must contain exactly the seven arm joints");
    auto snapshot = json::parse(read(directory + "/state.json"));
    std::map<std::string, double> measured = snapshot["joints"].get<std::map<std::string, double>>();
    for (const auto* joint : model->getActiveJointModels())
      for (const auto& name : joint->getVariableNames())
        if (!measured.count(name) || !std::isfinite(measured.at(name)))
          throw std::runtime_error("Missing measured model joint " + name);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(measured);
    state.update();
    planning_scene::PlanningScene scene(model);
    json report = {{"planning_group", expected}, {"execution_enabled", false},
                   {"workspace_obstacles_modelled", false}, {"measured_state_bounds_violations", bounds(state)},
                   {"measured_state_contacts", contacts(scene, state)}, {"captured_poses", json::array()}};
    for (int i = 3; i < argc; ++i)
    {
      auto pose = json::parse(read(argv[i]));
      auto configuration = measured;
      for (const auto& joint : expected) configuration[joint] = pose.at("joints").at(joint).get<double>();
      state.setVariablePositions(configuration);
      state.update();
      const auto& transform = state.getGlobalLinkTransform(pose.at("tip_name").get<std::string>());
      const auto p = pose.at("position").get<std::vector<double>>();
      const auto q = pose.at("orientation_xyzw").get<std::vector<double>>();
      if (p.size() != 3 || q.size() != 4 || pose.at("frame_id") != "base")
        throw std::runtime_error("Unsupported pose format");
      const double distance = (transform.translation() - Eigen::Vector3d(p[0], p[1], p[2])).norm();
      const double angle = Eigen::Quaterniond(transform.rotation()).angularDistance(Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized());
      if (distance > 1e-5 || angle > 1e-5)
        throw std::runtime_error("MoveIt FK does not match the controller capture: " + std::string(argv[i]));
      report["captured_poses"].push_back({{"file", argv[i]}, {"position_error_m", distance},
          {"orientation_error_rad", angle}, {"bounds_violations", bounds(state)}, {"contacts", contacts(scene, state)}});
    }
    // Deliberately insert a synthetic blocker at the hand. A positive contact
    // demonstrates that robot/environment checks are actually enabled.
    const auto transform = state.getGlobalLinkTransform("right_hand");
    moveit_msgs::msg::CollisionObject blocker;
    blocker.header.frame_id = "base";
    blocker.id = "model_check_blocker";
    blocker.operation = blocker.ADD;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {.12};
    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.orientation.w = 1.;
    sphere_pose.position.x = transform.translation().x();
    sphere_pose.position.y = transform.translation().y();
    sphere_pose.position.z = transform.translation().z();
    blocker.primitives = {sphere};
    blocker.primitive_poses = {sphere_pose};
    if (!scene.processCollisionObjectMsg(blocker)) throw std::runtime_error("Cannot insert test blocker");
    auto blocker_contacts = contacts(scene, state);
    bool detected = false;
    for (const auto& pair : blocker_contacts)
      if (pair[0] == blocker.id || pair[1] == blocker.id) detected = true;
    if (!detected) throw std::runtime_error("Collision checker missed the deliberate blocker");
    report["synthetic_blocker_detected"] = true;
    blocker.operation = blocker.REMOVE;
    scene.processCollisionObjectMsg(blocker);
    std::mt19937 generator(41);
    bool self_collision_detected = false;
    for (int sample = 0; sample < 200 && !self_collision_detected; ++sample)
    {
      for (const auto& joint : expected)
      {
        const auto& b = model->getVariableBounds(joint);
        state.setVariablePosition(joint, std::uniform_real_distribution<double>(b.min_position_, b.max_position_)(generator));
      }
      state.update();
      auto pairs = contacts(scene, state);
      if (!pairs.empty())
      {
        report["self_collision_test_contacts"] = pairs;
        self_collision_detected = true;
      }
    }
    if (!self_collision_detected) throw std::runtime_error("Could not verify self-collision detection");
    report["model_checks"] = "PASS";
    std::ofstream output(argv[2]);
    output << report.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write report");
    std::cout << report.dump(2) << '\n';
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "Model check: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
