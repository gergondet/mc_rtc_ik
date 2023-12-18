#include <mc_solver/TasksQPSolver.h>

#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <mc_control/ControllerServer.h>

#include <thread>

int main()
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  auto dt = 0.005;

  auto solver = mc_solver::TasksQPSolver(robots, dt);
  solver.updateNrVars();

  mc_control::ControllerServer server{dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}};
  mc_rtc::gui::StateBuilder gui;

  bool quit = false;
  gui.addElement({}, mc_rtc::gui::Button("Quit", [&]() { quit = true; }));

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

  // Add tasks
  const auto & postureTask = std::make_shared<mc_tasks::PostureTask>(solver, 0, 1.0, 1.0);
  const auto & leftFootTask =
      std::make_shared<mc_tasks::TransformTask>(robots->robot().frame("L_ANKLE_P_S"), 1000.0, 1000.0);
  const auto & rightFootTask =
      std::make_shared<mc_tasks::TransformTask>(robots->robot().frame("R_ANKLE_P_S"), 1000.0, 1000.0);
  const auto & rightHandTask =
      std::make_shared<mc_tasks::TransformTask>(robots->robot().frame("r_wrist"), 1000.0, 1000.0);
  solver.addTask(postureTask);
  solver.addTask(leftFootTask);
  solver.addTask(rightFootTask);
  solver.addTask(rightHandTask);
  leftFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(15.0)), Eigen::Vector3d(0.0, 0.1, 0.0)));
  rightFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(-15.0)), Eigen::Vector3d(0.0, -0.1, 0.0)));
  Eigen::VectorXd rightHandTaskWeight(6);
  rightHandTaskWeight << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
  rightHandTask->dimWeight(rightHandTaskWeight);

  gui.addElement(
      {}, mc_rtc::gui::Robot(robots->robot().name(), [&]() -> const mc_rbdyn::Robot & { return robots->robot(); }));

  size_t iter = 0;
  while(!quit)
  {
    auto now = std::chrono::high_resolution_clock::now();
    double t = static_cast<double>(iter++) * solver.dt();

    rightHandTask->target(
        sva::PTransformd(Eigen::Vector3d(0.4 + 0.2 * std::cos(t), 0.2 + 0.2 * std::sin(t), 0.8)));
    if(!solver.run())
    {
      mc_rtc::log::info("QP failed to run");
      return 1;
    }
    server.handle_requests(gui);
    server.publish(gui);
    std::this_thread::sleep_until(now + std::chrono::milliseconds(static_cast<long>(1000 * dt)));
  }

  return 0;
}
