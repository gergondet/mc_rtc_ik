#include "helpers.h"

#include <benchmark/benchmark.h>

template<typename SolverT>
static void BM_Solver(benchmark::State & state)
{
  auto robots = setupRobots();
  auto dt = 0.005;
  auto solver = std::make_shared<SolverT>(robots, dt);
  size_t iter = 0;
  for(auto _ : state)
  {
    double t = static_cast<double>(iter++) * dt;
    if(!solver->run(t))
    {
      mc_rtc::log::info("QP failed to run");
      break;
    }
  }
}

auto BM_Acceleration_TasksSolver = BM_Solver<AccelerationSolver<mc_solver::TasksQPSolver>>;
BENCHMARK(BM_Acceleration_TasksSolver)->Unit(benchmark::kMicrosecond);

auto BM_Acceleration_TVMSolver = BM_Solver<AccelerationSolver<mc_solver::TVMQPSolver>>;
BENCHMARK(BM_Acceleration_TVMSolver)->Unit(benchmark::kMicrosecond);

auto BM_Velocity_Solver = BM_Solver<VelocitySolver>;
BENCHMARK(BM_Velocity_Solver)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
