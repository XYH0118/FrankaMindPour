#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include "franka_mind_pour/mind_pour_controller.h"

using franka_mind_pour::MindPourController;
using franka_mind_pour::PourCommand;
using franka_mind_pour::PourState;

constexpr double kStart = 0.65;
constexpr double kDt = 0.001;
constexpr double kRad = 3.14159265358979323846 / 180.0;

void check(bool condition, const char* failure) {
  if (!condition) {
    std::cerr << failure << '\n';
    std::exit(1);
  }
}

struct Trial {
  hardware_interface::RobotHW robot;
  ros::NodeHandle node;
  MindPourController controller;
  double seconds = 1.0;
  double velocity = 0.0;

  Trial() {
    robot.joints.measured[6] = kStart;
    robot.joints.desired[6] = kStart;
    check(controller.init(&robot, node), "Controller init failed");
    controller.starting(ros::Time(1.0));
  }

  void tick(int8_t direction, bool update_feedback = true, double dt = kDt) {
    seconds += dt;
    const auto time = ros::Time(seconds);
    PourCommand command;
    command.header.stamp = time;
    command.command = direction;
    node.deliver(&command);
    const double previous_position = robot.joints.desired[6];
    controller.update(time, ros::Duration(dt));
    const double position = robot.joints.desired[6];
    const double new_velocity = (position - previous_position) / dt;
    check(std::abs(new_velocity - velocity) <= 200.0 * kRad * dt + 1e-8,
          "Commanded change of velocity exceeded the acceleration limit");
    velocity = new_velocity;
    if (update_feedback) {
      robot.joints.measured[6] = position;
    }
  }

  PourState state() const { return realtime_tools::RealtimePublisher<PourState>::last; }
};

void held_return(int8_t side, double jitter = 0.0, unsigned seed = 0) {
  Trial trial;
  std::mt19937 random(seed);
  std::uniform_real_distribution<double> period(kDt - jitter, kDt + jitter);
  for (int i = 0; i < 850; ++i) trial.tick(side);
  check(side * (trial.robot.joints.desired[6] - kStart) > 10 * kRad,
        "Fixture did not qualify the excursion");
  for (int i = 0; i < 1600; ++i) {
    trial.tick(-side, true, period(random));
    check(side * (trial.robot.joints.desired[6] - kStart) >= -1e-10,
          "Holding the return command crossed the start angle");
  }
  check(trial.state().state == PourState::STATE_DONE, "Held return never completed");
  check(trial.state().command == PourCommand::NONE, "DONE still reports rotation");
  check(std::abs(trial.robot.joints.desired[6] - kStart) < 1e-8,
        "Held return did not stop at the start angle");
  for (int i = 0; i < 150; ++i) trial.tick(side);
  check(std::abs(trial.robot.joints.desired[6] - kStart) < 1e-8,
        "A later command moved the joint after DONE");
}

int main() {
  held_return(1);
  held_return(-1);
  for (unsigned seed = 0; seed < 32; ++seed) {
    held_return(1, 0.0005, seed);
    held_return(-1, 0.0005, seed);
  }

  Trial small;
  for (int i = 0; i < 200; ++i) small.tick(1);
  for (int i = 0; i < 490; ++i) small.tick(-1);
  check(small.robot.joints.desired[6] < kStart - kRad &&
            small.state().state == PourState::STATE_ROTATING,
        "A return before the minimum excursion must allow crossing without ending the trial");

  Trial paused;
  for (int i = 0; i < 850; ++i) paused.tick(1);
  for (int i = 0; i < 400; ++i) paused.tick(-1);
  for (int i = 0; i < 300; ++i) paused.tick(0);
  const double paused_position = paused.robot.joints.desired[6];
  check(paused_position > kStart + 2 * kRad && paused.state().state == PourState::STATE_ROTATING,
        "Releasing the command away from home must pause the trial");
  for (int i = 0; i < 300; ++i) paused.tick(0);
  check(paused.robot.joints.desired[6] == paused_position, "A pause must not auto-return the joint");
  for (int i = 0; i < 1400; ++i) paused.tick(-1);
  check(paused.state().state == PourState::STATE_DONE, "Held return did not resume after a pause");

  Trial delayed;
  for (int i = 0; i < 850; ++i) delayed.tick(1);
  for (int i = 0; i < 1600; ++i) delayed.tick(-1, false);
  check(delayed.state().state == PourState::STATE_ROTATING &&
            std::abs(delayed.robot.joints.desired[6] - kStart) < 1e-8,
        "Completion must wait for measured position after the setpoint stops at home");
  for (int i = 0; i < 100; ++i) delayed.tick(1, false);
  check(std::abs(delayed.robot.joints.desired[6] - kStart) < 1e-8,
        "Changing commands while the measured position catches up moved the setpoint");
  delayed.robot.joints.measured[6] = kStart;
  for (int i = 0; i < 100; ++i) delayed.tick(-1);
  check(delayed.state().state == PourState::STATE_DONE, "Settled measured return did not finish");
}
