#pragma once

#include <memory>

#include "drake/math/wrap_to.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

#include <math.h>

namespace drake {
namespace examples {

using drake::systems::BasicVector;
using drake::systems::Context;

/// Feedback Cancellation controller. We want this acrobot act like a single pendulum.
/// Underactudated Course
/// https://deepnote.com/workspace/Underactuated-2ed1518a-973b-4145-bd62-1768b49956a8/project/52e7e101-429f-4aef-a373-e4cca7980cfe/notebook/intro-8ca11815e8354b658378157cd15c1725
///
/// @system
/// name: FBCancellationController
/// input_ports:
/// - joints_state
/// output_ports:
/// - joints_torque
/// @endsystem
///
/// @ingroup acrobot_systems
template <typename T>
class FBCancellationController : public systems::LeafSystem<T> {
 public:
  FBCancellationController(const multibody::MultibodyPlant<T>& plant)
      : plant_(plant), plant_context_(plant_.CreateDefaultContext())
    {
    this->DeclareVectorInputPort("joints_state", 4); 
    this->DeclareVectorOutputPort("joints_torque", 2, 
                                  &FBCancellationController<T>::CalcControlTorque);

    nv_ = plant_.num_velocities();
    drake::log()->info("nv_ {}", nv_);

    int np_ = plant_.num_positions();
    drake::log()->info("np_ {}", np_);

    int ns_ = plant_.num_multibody_states();
    drake::log()->info("ns_ {}", ns_);

    int ndofs_ = plant_.num_actuated_dofs();
    drake::log()->info("ndofs_ {}", ndofs_);
  }

  void CalcControlTorque(const Context<T>& context, BasicVector<T>* torque) const {

    // We do not have access to the context of the another system here.
    // we do not have access to the root_context or diagram either.
    // So we get the state from inpout of THIS system and update the 
    // plant context
    const Eigen::VectorXd& state = this->EvalVectorInput(context, 0)->value();
    plant_.SetPositionsAndVelocities(plant_context_.get(), state);
    drake::log()->info("state:\n{}", fmt_eigen(state));

    // the context argument of this function is the root context related to diagram, but
    // what we want is the one realted to plant which is plant_context_
    Eigen::MatrixXd M(nv_, nv_);
    plant_.CalcMassMatrix(*plant_context_, &M);
    drake::log()->info("M:\n{}", fmt_eigen(M));

    Eigen::VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(*plant_context_);
    drake::log()->info("tau_g:\n{}", fmt_eigen(tau_g));
    
    // bias term Cv = C(q,qdot)*qdot contains: coriolis, cetripetal and gyroscopic
    Eigen::VectorX<T> Cv(2);
    plant_.CalcBiasTerm(*plant_context_, &Cv);
    drake::log()->info("Cv:\n{}", fmt_eigen(Cv));

    Eigen::Vector2d q; q << state[0], state[1];
    Eigen::Vector2d qdot; qdot << state[2], state[3];

    double g = -9.8, kp = 1, kd = 0.1, length = 1.0, b = 0.1;

    Eigen::VectorXd qddot(2);
    qddot <<  g/length * std::sin(q[0]) - b * qdot[0],
              -kp * q[1] - kd * qdot[1] ;

    drake::log()->info("qddot:\n{}", fmt_eigen(qddot));

    Eigen::VectorXd u(2);
    u = Cv - tau_g + M * qddot;
    
    torque->set_value(u);
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  // The implementation above is (and must remain) careful to not store hidden
  // state in here.  This is only used to avoid runtime allocations.

  // plant's context. its a copy of the context, we need to update it by giving 
  // state to it, so it can act like the realone
  const std::unique_ptr<Context<T>> plant_context_;
  int nv_;
};

}  // namespace examples
}  // namespace drake
