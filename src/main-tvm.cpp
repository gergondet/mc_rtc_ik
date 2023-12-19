#include "helpers.h"

int main()
{
  auto robots = setupRobots();
  auto dt = 0.005;

  mc_control::ControllerServer server{dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}};
  mc_rtc::gui::StateBuilder gui;

  bool quit = false;
  gui.addElement({}, mc_rtc::gui::Button("Quit", [&]() { quit = true; }));

  auto solver = std::make_shared<AccelerationSolver<mc_solver::TVMQPSolver>>(robots, dt);

  gui.addElement(
      {}, mc_rtc::gui::Robot(robots->robot().name(), [&]() -> const mc_rbdyn::Robot & { return robots->robot(); }));

  size_t iter = 0;
  gui.addElement({}, mc_rtc::gui::NumberInput(
                         "iter", [&]() { return iter; }, [](double) {}));
  while(!quit)
  {
    auto now = std::chrono::high_resolution_clock::now();
    double t = static_cast<double>(iter++) * dt;

    if(!solver->run(t))
    {
      return 1;
    }

    server.handle_requests(gui);
    server.publish(gui);
    std::this_thread::sleep_until(now + std::chrono::milliseconds(static_cast<long>(1000 * dt)));
  }

  return 0;
}
