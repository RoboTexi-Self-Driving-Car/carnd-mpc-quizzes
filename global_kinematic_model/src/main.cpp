// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"

using Eigen::VectorXd;

//
// Helper functions
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// Return the next state.
VectorXd globalKinematic(const VectorXd &state,
                         const VectorXd &actuators, double dt);

int main() {
  // [x, y, psi, v]
  VectorXd state(4);
  // [delta, v]
  VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  // should be [0.212132, 0.212132, 0.798488, 1.3]
  auto next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}

VectorXd globalKinematic(const VectorXd &state,
                         const VectorXd &actuators, double dt) {
  // Create a new vector for the next state.
  VectorXd next_state(state.size());

  /**
   * Done: Implement the global kinematic model,
   *   to return the next state from the inputs.
   */

  // NOTE: state is [x, y, psi, v] and actuators is [delta, a]

  // Restore the initial state variables
  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];

  // Restore the control inputs
  double delta = actuators[0];
  double a = actuators[1];

  // Compute the next state with the global kinematic model
  double x1 = x0 + v0 * cos(psi0) * dt;
  double y1 = y0 + v0 * sin(psi0) * dt;
  double psi1 = psi0 + v0 / Lf * delta * dt;
  double v1 = v0 + a * dt;

  next_state << x1, y1, psi1, v1;

  return next_state;
}
