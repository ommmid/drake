#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/acrobot_state.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/drake_path.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the (passive) Acrobot.  Run meldis to see
// the animated result.

DEFINE_double(simulation_sec, 10.0,
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");


int do_main() {

  systems::DiagramBuilder<double> builder;
  
  // [plant]: dynamics equations. to calculate the nexts state for given actuation
  // basically compute the physics
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  acrobot->set_name("acrobot");
  
  // [scene graph]
  // scene graph is where we introduce geometry of objects. they can be static (anchored)
  // or dynamic which their pose depends on Context. 
  // N geometries means N pose/config input ports to scene graph
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  
  // [acrobot geometry]. This is usually done by MultiBodyPlant but we want 
  // to view the explicit equations. 
  AcrobotGeometry::AddToBuilder(
      &builder, acrobot->get_output_port(0), scene_graph);
  
  // visualization
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  //
  auto diagram = builder.Build();

  const std::string drawing = diagram->GetGraphvizString();
  bool res = drake::writeDot(drawing, "/home/omid/test_dir/run_passive.dot");
  drake::log()->info("res: {}", res);

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(*acrobot,
                                          &simulator.get_mutable_context());

  // OMID: applied torque should be zero. We do not create a MultiBodyPlant here but 
  const double tau = 0;
  acrobot->GetInputPort("elbow_torque").FixValue(&acrobot_context, tau);

  const Matrix2<double> mass_matrix = acrobot->MassMatrix(acrobot_context);
  drake::log()->info("Mass Matrix: {}", fmt_eigen(mass_matrix));

  // Set an initial condition that is sufficiently far from the downright fixed
  // point.
  AcrobotState<double>* x0 = dynamic_cast<AcrobotState<double>*>(
      &acrobot_context.get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(1.0);
  x0->set_theta2(1.0);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::do_main();
}
