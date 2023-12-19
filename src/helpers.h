#pragma once

#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>

#include <mc_tvm/PostureFunction.h>
#include <mc_tvm/Robot.h>
#include <mc_tvm/PositionFunction.h>
#include <mc_tvm/TransformFunction.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <mc_control/ControllerServer.h>

#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/scheme/WeightedLeastSquares.h>
#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/task_dynamics/Proportional.h>

#include <thread>

std::shared_ptr<mc_rbdyn::Robots> setupRobots();

template<typename T>
struct AccelerationSolver
{
  AccelerationSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double dt);

  bool run(double t);

  T solver;

  std::shared_ptr<mc_tasks::PositionTask> rightHandTask;
};

extern template struct AccelerationSolver<mc_solver::TasksQPSolver>;
extern template struct AccelerationSolver<mc_solver::TVMQPSolver>;

struct VelocitySolver
{
  VelocitySolver(std::shared_ptr<mc_rbdyn::Robots> robots, double dt);

  bool run(double t);

  /** Robots */
  std::shared_ptr<mc_rbdyn::Robots> robots;
  /** Timestep */
  double dt;
  /** Control problem */
  tvm::LinearizedControlProblem problem;
  /** Solver scheme */
  tvm::scheme::WeightedLeastSquares solver{tvm::solver::DefaultLSSolverOptions{}};

  /** Right hand function */
  std::shared_ptr<mc_tvm::PositionFunction> rightHand;

};
