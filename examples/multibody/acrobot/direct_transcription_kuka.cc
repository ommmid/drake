#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/unused.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/planning/trajectory_optimization/direct_transcription.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include <iostream>

namespace drake {

using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::RevoluteJoint;
using systems::Context;
using Eigen::Vector2d;
using drake::planning::trajectory_optimization::DirectTranscription;
using trajectories::PiecewisePolynomial;
using solvers::Solve;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");


int do_main() {
    systems::DiagramBuilder<double> builder;

    const double time_step = FLAGS_time_stepping ? 1.0e-2 : 0.0;
    auto [plant, scene_graph] =
        AddMultibodyPlantSceneGraph(&builder, time_step);

    // Make and add the plant model.
    const std::string dp_underactuated_url =
        "package://drake_models/iiwa_description/urdf/planar_iiwa14_spheres_dense_elbow_collision.urdf";
    Parser parser(&plant);
    parser.AddModelsFromUrl(dp_underactuated_url);
    // weld the base of the robot to world
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
    plant.SetUseSampledOutputPorts(false);
    plant.Finalize();

    // Set default positions:
    RevoluteJoint<double>& joint_2 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_2");
    RevoluteJoint<double>& joint_4 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_4");
    RevoluteJoint<double>& joint_6 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_6");

    joint_2.set_default_angle(0.1);
    joint_4.set_default_angle(-1.02);
    joint_6.set_default_angle(1.6);

/*
  auto controller = builder.AddSystem<drake::examples::FBCancellationController<double>>(acrobot);
  controller->set_name("controller");
  builder.Connect(acrobot.get_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot.get_actuation_input_port());
*/
    visualization::AddDefaultVisualization(&builder);
    auto diagram = builder.Build();
    auto diagram_context = diagram->CreateDefaultContext(); // unique pointer, unique_point.get() return the raw pointer
    
    const std::string drawing = diagram->GetGraphvizString();
    bool res = drake::writeDot(drawing, "/home/omid/test_dir/dt_kuka.dot");
    drake::log()->info("res: {}", res);

    const Context<double>& plant_context = plant.GetMyContextFromRoot(*diagram_context);
    unused(plant_context);

    int num_q = plant.num_positions();
    unused(num_q);
    drake::log()->info("plant number of position and velocity: {}, {}", plant.num_positions(), plant.num_velocities());

    DirectTranscription dirtran(
            &plant, 
            plant_context, 
            100,
            plant.get_actuation_input_port().get_index()
    );

    auto& prog = dirtran.prog();
    drake::log()->info("num of var: {}", prog.num_vars());
    drake::log()->info("decision variables: {}", prog.decision_variables().size());
    
    const solvers::VectorXDecisionVariable& u = dirtran.input();
    drake::log()->info("size of u: {}", u.size());
    
    // boundary conditions
    // prog.AddBoundingBoxConstraint(q0, q0, dirtran.initial_state())
    // prog.AddBoundingBoxConstraint(qf, qf, dirtran.final_state())
    dirtran.AddConstraintToAllKnotPoints(u[0] <= 1);
    dirtran.AddConstraintToAllKnotPoints(u[0] >= -1);

    const double R = 10;  // Cost on input "effort".
    dirtran.AddRunningCost(R * u.transpose() * u);
    
    
    // initial solution
    // initial_u = PiecewisePolynomial.ZeroOrderHold([0, 0.3*21],
    //                                                   np.zeros((1, 2)))
    // initial_x = PiecewisePolynomial()
    // dirtran.SetInitialTrajectory(initial_u, initial_x)

    // const double timespan_init = 4;
    // auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
    //             {0, timespan_init},
    //             {initial_state.get_value(), final_state.get_value()}
    // );

    // dirtran->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);

    // solvers::SnoptSolver snopt_solver;
    // if (!(snopt_solver.is_available() && snopt_solver.is_enabled())) {
    //     drake::log()->warn(
    //         "SNOPT is unavailable or disabled; this test shall be skipped.");
    //     return 0;
    // }
    // const auto result = snopt_solver.Solve(dirtran.prog(), {}, {});

    const solvers::MathematicalProgramResult result = Solve(dirtran.prog());
        
    // systems::Simulator<double> simulator(*diagram);
    // simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

    // simulator.get_mutable_context().SetTime(0.0);
    // simulator.Initialize();
    // simulator.AdvanceTo(FLAGS_simulation_time);

    return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant with "
      "LQR stabilization. "
      "Launch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::acrobot::do_main();
}
