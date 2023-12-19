#include "helpers.h"

std::shared_ptr<mc_rbdyn::Robots> setupRobots()
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  std::vector<std::string> jname_list = {
      "R_HIP_P",   "R_HIP_R",   "R_HIP_Y",   "R_KNEE",       "R_ANKLE_R",    "R_ANKLE_P",    "L_HIP_P",
      "L_HIP_R",   "L_HIP_Y",   "L_KNEE",    "L_ANKLE_R",    "L_ANKLE_P",    "WAIST_Y",      "WAIST_P",
      "WAIST_R",   "NECK_Y",    "NECK_R",    "NECK_P",       "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y",
      "R_ELBOW_P", "R_ELBOW_Y", "R_WRIST_R", "R_WRIST_Y",    "R_UTHUMB",     "R_LTHUMB",     "R_UINDEX",
      "R_LINDEX",  "R_ULITTLE", "R_LLITTLE", "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P",
      "L_ELBOW_Y", "L_WRIST_R", "L_WRIST_Y", "L_UTHUMB",     "L_LTHUMB",     "L_UINDEX",     "L_LINDEX",
      "L_ULITTLE", "L_LLITTLE"};
  std::vector<double> jpos_list = {-0.38, -0.01, 0., 0.72, -0.01, -0.33,  -0.38, 0.02, 0.,    0.72, -0.02, -0.33, 0.,
                                   0.13,  0.,    0., 0.,   0.,    -0.052, -0.17, 0.,   -0.52, 0.,   0.,    0.,    0.,
                                   0.,    0.,    0., 0.,   0.,    -0.052, 0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,
                                   0.,    0.,    0., 0.,   0.,    0.,     0.,    0.,   0.,    0.,   0.};
  const auto & mb = robots->robot().mb();
  auto & mbc = robots->robot().mbc();
  for(size_t i = 0; i < jname_list.size(); i++)
  {
    int jointIdx = mb.jointIndexByName(jname_list[i]);
    mbc.q[jointIdx][0] = jpos_list[i];
  }
  robots->robot().forwardKinematics();
  robots->robot().forwardVelocity();
  return robots;
}

template<typename T>
AccelerationSolver<T>::AccelerationSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double dt) : solver(robots, dt)
{
  if constexpr(std::is_same_v<T, mc_solver::TasksQPSolver>) { solver.updateNrVars(); }
  // Add tasks
  const auto & postureTask = std::make_shared<mc_tasks::PostureTask>(solver, 0, 1.0, 1.0);
  const auto & leftFootTask =
      std::make_shared<mc_tasks::TransformTask>(robots->robot().frame("L_ANKLE_P_S"), 100.0, 1000.0);
  const auto & rightFootTask =
      std::make_shared<mc_tasks::TransformTask>(robots->robot().frame("R_ANKLE_P_S"), 100.0, 1000.0);
  rightHandTask = std::make_shared<mc_tasks::PositionTask>(robots->robot().frame("r_wrist"), 100.0, 1000.0);

  solver.addTask(postureTask);
  solver.addTask(leftFootTask);
  solver.addTask(rightFootTask);
  solver.addTask(rightHandTask);

  leftFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(15.0)), Eigen::Vector3d(0.0, 0.1, 0.0)));
  rightFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(-15.0)), Eigen::Vector3d(0.0, -0.1, 0.0)));
}

template<typename T>
bool AccelerationSolver<T>::run(double t)
{
  rightHandTask->position(Eigen::Vector3d(0.4 + 0.2 * std::cos(t), 0.2 + 0.2 * std::sin(t), 0.8));
  if(!solver.run())
  {
    mc_rtc::log::info("QP failed to run");
    return false;
  }
  return true;
}

template struct AccelerationSolver<mc_solver::TasksQPSolver>;
template struct AccelerationSolver<mc_solver::TVMQPSolver>;

VelocitySolver::VelocitySolver(std::shared_ptr<mc_rbdyn::Robots> robots, double dt) : robots(robots), dt(dt)
{
  auto & robot = robots->robot();
  auto & tvm_robot = robot.tvmRobot();

  auto addTask = [&](auto error, double stiffness, double weight)
  {
    tvm::requirements::SolvingRequirements reqs{tvm::requirements::PriorityLevel(1),
                                                tvm::requirements::Weight(weight)};
    problem.add(error == 0., tvm::task_dynamics::P(stiffness), reqs);
  };

  auto posture = std::make_shared<mc_tvm::PostureFunction>(robot);
  addTask(posture, 1.0, 1.0);

  auto leftFoot = std::make_shared<mc_tvm::TransformFunction>(robot.frame("L_ANKLE_P_S"));
  leftFoot->pose(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(15.0)), Eigen::Vector3d(0.0, 0.1, 0.0)));
  addTask(leftFoot, 100.0, 1000.0);

  auto rightFoot = std::make_shared<mc_tvm::TransformFunction>(robot.frame("R_ANKLE_P_S"));
  rightFoot->pose(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(-15.0)), Eigen::Vector3d(0.0, -0.1, 0.0)));
  addTask(rightFoot, 100.0, 1000.0);

  rightHand = std::make_shared<mc_tvm::PositionFunction>(robot.frame("r_wrist"));
  addTask(rightHand, 100.0, 1000.0);
}

bool VelocitySolver::run(double t)
{
  auto & robot = robots->robot();
  auto & tvm_robot = robot.tvmRobot();
  rightHand->position(Eigen::Vector3d(0.4 + 0.2 * std::cos(t), 0.2 + 0.2 * std::sin(t), 0.8));
  if(!solver.solve(problem))
  {
    mc_rtc::log::info("Solver failed to run");
    return false;
  }
  rbd::vectorToParam(tvm_robot.alpha()->value(), robot.alpha());
  robot.eulerIntegration(dt);
  robot.forwardKinematics();
  robot.forwardVelocity();
  return true;
}
