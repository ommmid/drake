#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/visualization/visualization_config_functions.h"

#include <iostream>
#include "drake/examples/multibody/acrobot/fb_cancellation_controller.h"
#include "drake/common/drake_path.h"

namespace drake {

using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::RevoluteJoint;
using systems::Context;
using Eigen::Vector2d;

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

  const double time_step = FLAGS_time_stepping ? 1.0e-3 : 0.0;
  auto [acrobot, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, time_step);

  // Make and add the acrobot model.
  const std::string dp_underactuated_url =
      "package://drake/multibody/benchmarks/acrobot/double_pendulum_underactuated.sdf";
  Parser parser(&acrobot);
  parser.AddModelsFromUrl(dp_underactuated_url);

  // weld the base of the robot to world
//   acrobot.WeldFrames(acrobot.world_frame(), acrobot.GetFrameByName("base"));
  acrobot.Finalize();

  RevoluteJoint<double>& shoulder =
      acrobot.GetMutableJointByName<RevoluteJoint>("shoulder");
  RevoluteJoint<double>& elbow =
      acrobot.GetMutableJointByName<RevoluteJoint>("elbow");

  // Drake's parser will default the name of the actuator to match the name of
  // the joint it actuates.
  const JointActuator<double>& actuator =
      acrobot.GetJointActuatorByName("elbow");
  DRAKE_DEMAND(actuator.joint().name() == "elbow");

  // For this example the controller's model of the plant exactly matches the
  // plant to be controlled (in reality there would always be a mismatch).
//   auto controller = builder.template AddSystem<drake::examples::FBCancellationController<double>>(acrobot);
  auto controller = builder.AddSystem<drake::examples::FBCancellationController<double>>(acrobot);
  controller->set_name("controller");
  builder.Connect(acrobot.get_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot.get_actuation_input_port());

  visualization::AddDefaultVisualization(&builder);
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext(); // unique pointer, unique_point.get() return the raw pointer
  
  const std::string drawing = diagram->GetGraphvizString();
  bool res = drake::writeDot(drawing, "/home/omid/test_dir/fb.dot");
  drake::log()->info("res: {}", res);

  
  RandomGenerator generator;

  // Setup distribution for random initial conditions.
  std::normal_distribution<symbolic::Expression> gaussian;
  // shoulder.set_random_angle_distribution(M_PI + 0.02*gaussian(generator));
  // elbow.set_random_angle_distribution(0.05*gaussian(generator));
  shoulder.set_default_angle(M_PI/3);
  elbow.set_default_angle(0.0);

  // [OMID] I dont know why the following doe not work for setting initial state. Figure out why?
  // Context<double>& plant_context = acrobot.GetMyMutableContextFromRoot(diagram_context.get());
  // auto state = plant_context.get_mutable_discrete_state(0).get_mutable_value();
  // state[0] = M_PI/4;
  // state[1] = 2.0;
  // state[2] = 0.0;
  // state[3] = 0.0;

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  simulator.get_mutable_context().SetTime(0.0);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

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
