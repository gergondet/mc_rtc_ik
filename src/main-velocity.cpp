#include "helpers.h"

int main()
{
  auto robots = setupRobots();
  auto & robot = robots->robot();

  auto dt = 0.005;

  mc_control::ControllerServer server{dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}};
  mc_rtc::gui::StateBuilder gui;

  auto solver = std::make_shared<VelocitySolver>(*robots, dt);

  size_t iter = 0;
  bool quit = false;
  bool stepByStep = false;
  bool run = false;
  gui.addElement({}, mc_rtc::gui::Checkbox("Step by step", stepByStep),
                 mc_rtc::gui::Button("Step", [&]() { run = true; }),
                 mc_rtc::gui::NumberInput(
                     "iter", [&]() { return iter; }, [](double) {}),
                 mc_rtc::gui::Robot("robot", [&]() -> const mc_rbdyn::Robot & { return robot; }),
                 mc_rtc::gui::Button("Quit", [&]() { quit = true; }));

  mc_rtc::log::info("Problem variables:");
  for(const auto & v : solver->problem.variables()) { mc_rtc::log::info("- {}", v->name()); }

  while(!quit)
  {
    auto now = std::chrono::high_resolution_clock::now();
    if(!stepByStep || (stepByStep && run))
    {
      double t = static_cast<double>(iter++) * dt;
      run = false;
      if(!solver->run(t))
      {
        mc_rtc::log::info("QP failed to run");
        return 1;
      }
    }
    server.handle_requests(gui);
    server.publish(gui);
    std::this_thread::sleep_until(now + std::chrono::milliseconds(static_cast<long>(1000 * dt)));
  }

  return 0;
}
