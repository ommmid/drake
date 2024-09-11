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

/// The Spong acrobot swing-up controller as described in:
///   Spong, Mark W. "Swing up control of the acrobot." Robotics and Automation,
///   1994. Proceedings., 1994 IEEE International Conference on. IEEE, 1994.
///
/// Note that the Spong controller works well on the default set of parameters,
/// which Spong used in his paper. In contrast, it is difficult to find a
/// functional set of gains to stabilize the robot about its upright fixed
/// point using the parameters of the physical robot we have in lab.
///
/// @system
/// name: FBCancellationController
/// input_ports:
/// - acrobot_state
/// output_ports:
/// - elbow_torque
/// @endsystem
///
/// @ingroup acrobot_systems
template <typename T>
class FBCancellationController : public systems::LeafSystem<T> {
 public:
  FBCancellationController(const multibody::MultibodyPlant<T>& plant)
      : plant_(plant),
        context_(plant_.CreateDefaultContext())
    {
    this->DeclareVectorInputPort("joints_state", 4); 
    this->DeclareVectorOutputPort("joints_torque", 1, 
                                  &FBCancellationController<T>::CalcControlTorque);

    

    // auto state_value =
    //   context_->get_mutable_discrete_state(0).get_mutable_value();
    
    // state->set_theta1(0.0);
    // state->set_theta2(0.0);
    // state->set_theta1dot(0.0);
    // state->set_theta2dot(0.0);
  }

  void CalcControlTorque(const Context<T>& context, BasicVector<T>* torque) const {

    // const Vector4<T> x0(M_PI, 0, 0, 0);
    // Vector4<T> x = state.CopyToVector();
    // // Wrap theta1 and theta2.
    // x(0) = math::wrap_to(x(0), 0., 2. * M_PI);
    // x(1) = math::wrap_to(x(1), -M_PI, M_PI);

    // const T cost = (x - x0).dot(S_ * (x - x0));
    // T u;

    // const Matrix2<T> M = plant_.MassMatrix(*context_);
    // const Vector2<T> bias = plant_.DynamicsBiasTerm(*context_);
    // const Matrix2<T> M_inverse = M.inverse();

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

    Eigen::VectorXd val(1);
    val << (1.0 + context.get_time() - context.get_time());
    torque->set_value(val);
    // drake::log()->info("size: {}", torque->size());
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  // The implementation above is (and must remain) careful to not store hidden
  // state in here.  This is only used to avoid runtime allocations.
  const std::unique_ptr<Context<T>> context_;

};

}  // namespace examples
}  // namespace drake
