#pragma once

#include <memory>

#include "drake/math/wrap_to.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace examples {

using drake::systems::BasicVector;
using drake::systems::Context;

/// Feedback Cancellation controller. We want this acrobot act like a single pendulum.
///
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
      : plant_(plant), context_(plant_.CreateDefaultContext())
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

    // auto state_value =
    //   context_->get_mutable_discrete_state(0).get_mutable_value();
    
    // state->set_theta1(0.0);
    // state->set_theta2(0.0);
    // state->set_theta1dot(0.0);
    // state->set_theta2dot(0.0);
  }

  void CalcControlTorque(const Context<T>& context, BasicVector<T>* torque) const {

    // the context argument of this function is the root context related to diagram, but
    // what we want is the one realted to plant which is context_
    Eigen::MatrixXd M(nv_, nv_);
    plant_.CalcMassMatrix(*context_, &M);
    // const Vector2<T> bias = plant_.DynamicsBiasTerm(*context_);
    // const Matrix2<T> M_inverse = M.inverse();
    drake::log()->info("M:\n{}", fmt_eigen(M));

    Eigen::VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(*context_);
    drake::log()->info("tau_g:\n{}", fmt_eigen(tau_g));
    
    // bias term Cv = C(q,qdot)*qdot contains: coriolis, cetripetal and gyroscopic
    Eigen::VectorX<T> Cv(2);
    plant_.CalcBiasTerm(*context_, &Cv);
    drake::log()->info("Cv:\n{}", fmt_eigen(Cv));


    // Cv - tau_g + 

    // // controller gains
    // const T& k_e = get_parameters(context).k_e();
    // const T& k_p = get_parameters(context).k_p();
    // const T& k_d = get_parameters(context).k_d();

    // const T PE = plant_.EvalPotentialEnergy(*context_);
    // const T KE = plant_.EvalKineticEnergy(*context_);
    // const T E = PE + KE;
    // const T E_desired =
    //     (p.m1() * p.lc1() + p.m2() * (p.l1() + p.lc2())) * p.gravity();
    // const T E_tilde = E - E_desired;
    // const T u_e = -k_e * E_tilde * state.theta2dot();

    // const T y = -k_p * state.theta2() - k_d * state.theta2dot();
    // T a3 = M_inverse(1, 1), a2 = M_inverse(0, 1);
    // T u_p = (a2 * bias(0) + y) / a3 + bias(1);

    // u = u_e + u_p;

    // // Saturation.
    // const T ku_upper_bound = 20;
    // const T ku_lower_bound = -20;
    // if (u >= ku_upper_bound) u = ku_upper_bound;
    // if (u <= ku_lower_bound) u = ku_lower_bound;

    // Eigen::Vector2d u(0.1,0.4);
    // torque->set_value(u);

    Eigen::VectorXd val(2);
    val << (0.1 + context.get_time() - context.get_time()), 0.2;
    torque->set_value(val);
    // drake::log()->info("size: {}", torque->size());
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  // The implementation above is (and must remain) careful to not store hidden
  // state in here.  This is only used to avoid runtime allocations.

  const std::unique_ptr<Context<T>> context_;
  int nv_;
};

}  // namespace examples
}  // namespace drake
