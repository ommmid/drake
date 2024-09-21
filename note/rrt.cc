/// @file drake_ompl_demo.cc
// @author: David M.S. Johnson <dave@dexai.com>
// https://github.com/DexaiRobotics/drake-torch/blob/master/examples/drake-ompl/src/drake_ompl_demo.cc

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/unused.h"
#include <drake/math/rigid_transform.h>

using hr_clock = std::chrono::high_resolution_clock;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraph;
using drake::geometry::SignedDistancePair;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_double(sim_dt, 3e-3,
              "The time step to use for MultibodyPlant model "
              "discretization.");
DEFINE_double(discretization, 1e-2,
              "The OMPL RealVectorSpace "
              "discretization in radians.");
DEFINE_double(collision_tolerance, 1e-3,
              "The collision tolerance (meters) to use for OMPL planning.");
DEFINE_double(max_distance, 0.5,
              "The maximum distance in meters at which distance data is "
              "reported during collision checking.");
DEFINE_double(solve_time, 15.0,
              "The time to spend trying to solve the OMPL planning problem.");
DEFINE_double(simplify_time, 10.0,
              "The time to spend to try and simplify the solution to the OMPL "
              "planning problem.");
DEFINE_int32(pause_msec, 150,
             "Determines Playback speed. Time to pause between displaying each "
             "solution state in msec.");
DEFINE_int32(cycles_to_display, 1,
             "Determines how long the solution is played back. Each cycle is "
             "num_interpolated_states * pause_msec * 2 /1000 seconds long.");
DEFINE_int32(num_interpolated_states, 50,
             "Determines Playback speed. Number of states in the OMPL "
             "trajectory to display.");

Eigen::VectorXd OMPLStateToEigen(const ompl::base::State *state, int size);
Eigen::VectorXd OMPLStateToEigen(const ompl::base::State *state, int size) {
  Eigen::VectorXd state_vector = Eigen::VectorXd::Zero(size);
  const ompl::base::RealVectorStateSpace::StateType *pos =
      state->as<ompl::base::RealVectorStateSpace::StateType>();
  for (int i = 0; i < size; i++) {
    state_vector(i) = (*pos)[i];
  }
  return state_vector;
}

std::vector<double> eigen_to_vec(Eigen::VectorXd e);
std::vector<double> eigen_to_vec(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // immediately start a timer
  auto t_begin = hr_clock::now();
  // build the basic drake system
  drake::systems::DiagramBuilder<double> builder;
  // create a scene graph that contains all the geometry of the system.
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);
  scene_graph.set_name("scene_graph");

  // define our geometry source
  // const std::string robot_urdf_filepath{"iiwa14_boxes.urdf"};
  const char *kModelPath = "package://drake_models/iiwa_description/"
                           "urdf/iiwa14_spheres_dense_elbow_collision.urdf";
  const std::string robot_root_link_in_urdf{"base"};
  // const std::string urdf =
  //     (!FLAGS_urdf.empty() ? FLAGS_urdf
  //                          : drake::FindResourceOrThrow(kModelPath));

  // import robot
  ModelInstanceIndex robot_model_idx_ = Parser(&plant).AddModelsFromUrl(kModelPath)[0];
  
  // obtain vector of Body Indices for each model for the Geometry Sets
  std::vector<BodyIndex> robot_body_indices = plant.GetBodyIndices(robot_model_idx_);
  std::vector<const Body<double> *> robot_bodies;
  for (size_t k = 0; k < robot_body_indices.size(); k++) {
    robot_bodies.push_back(&plant.get_body(robot_body_indices[k]));
  }

  auto &child_frame =
      plant.GetFrameByName(robot_root_link_in_urdf, robot_model_idx_);
  plant.WeldFrames(plant.world_frame(), child_frame);

  // add gripper to plant and return modelinstance
   const std::string wsg_model_path_ =  "package://drake_models/"
                                        "wsg_50_description/sdf/schunk_wsg_50.sdf"; 
  const ModelInstanceIndex wsg_instance =
        Parser(plant).AddModels(wsg_model_path_).at(0);

  // import shelves in the scene
  auto parser = Parser(&plant);
  ModelInstanceIndex bin = parser.AddModelsFromUrl("file:///home/omid/drake/note/shelves.sdf")[0];
  auto transform = RigidTransformd(drake::Vector3<double>{0.88, 0, 0.4});
  auto& shelves = plant.GetFrameByName("shelves_body", bin);
  plant.WeldFrames( plant.world_frame(), shelves, transform);

  // Now the model is complete.
  plant.Finalize();

  auto robot_dof_ = plant.num_positions(robot_model_idx_);
  std::cerr << "RobotCollisionChecker: robot_dof_: " << robot_dof_ << std::endl;

  // Boundary conditions
  Eigen::VectorXd start_conf = Eigen::VectorXd::Zero(robot_dof_);
  start_conf << 0.0, 0.40236988, 0.0, -1.36484125, 0.0, -0.1, -M_PI/2;
  Eigen::VectorXd goal_conf = Eigen::VectorXd::Zero(robot_dof_);
  goal_conf << 0.0, 0.60544649, 0.0, -1.75551969, 0.0, -0.69582573, -M_PI/2;

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  // connect the visualizer and make the context
  drake::visualization::AddDefaultVisualization(&builder);
  std::unique_ptr<drake::systems::Diagram<double>> diagram_ = builder.Build();

  // auto simulator = std::make_unique<drake::systems::Simulator<double>>(*diagram_);
  drake::systems::Simulator<double> simulator(*diagram_);

  // auto diagram_context = diagram_->CreateDefaultContext();
  drake::systems::Context<double>& diagram_context = simulator.get_mutable_context();
  // diagram_->SetDefaultContext(diagram_context.get());

  // auto scene_context = scene_graph.AllocateContext();
  // auto output = diagram_->AllocateOutput();

  Context<double>& plant_context = plant.GetMyMutableContextFromRoot(&diagram_context);
  // Context<double>& plant_context = diagram_->GetMutableSubsystemContext(plant, &diagram_context);
  drake::unused(plant_context);
  drake::unused(t_begin);

  // initialize to a default position (may be in collision)
  plant.SetPositions(&plant_context, start_conf);

  // OR:
  // RevoluteJoint<double>& joint_1 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_1");
  // RevoluteJoint<double>& joint_2 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_2");
  // RevoluteJoint<double>& joint_3 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_3");
  // RevoluteJoint<double>& joint_4 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_4");
  // RevoluteJoint<double>& joint_5 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_5");
  // RevoluteJoint<double>& joint_6 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_6");
  // RevoluteJoint<double>& joint_7 = plant.GetMutableJointByName<RevoluteJoint>("iiwa_joint_7");
  // joint_1.set_default_angle(start_conf[0]);
  // joint_2.set_default_angle(start_conf[1]);
  // joint_3.set_default_angle(start_conf[2]);
  // joint_4.set_default_angle(start_conf[3]);
  // joint_5.set_default_angle(start_conf[4]);
  // joint_6.set_default_angle(start_conf[5]);
  // joint_7.set_default_angle(start_conf[6]);

  diagram_->ForcedPublish(diagram_context);

  // Dont run simulation here, we jut want to see the robot at the given configuration, that is it.
  // uncomment to see the robot in initial state
  // while(true){
  //   std::this_thread::sleep_for(
  //           std::chrono::milliseconds(500));
  // }
  // return 0;
  
  // setup the OMPL planning problem
  // ompl::msg::noOutputHandler(); // uncomment to remove OMPL messages

  // create the vector space and use SimpleSetup
  auto space_ = std::make_shared<ob::RealVectorStateSpace>(robot_dof_);
  DRAKE_DEMAND(space_ && "NULL space_");
  ob::RealVectorBounds bounds(space_->getDimension());
  VectorXd pos_upper_limits = plant.GetPositionUpperLimits();
  VectorXd pos_lower_limits = plant.GetPositionLowerLimits();
  DRAKE_DEMAND(pos_upper_limits.size() == robot_dof_);
  DRAKE_DEMAND(pos_lower_limits.size() == robot_dof_);
  for (uint i = 0; i < uint(robot_dof_); i++) {
    bounds.setHigh(i, pos_upper_limits(i));
    bounds.setLow(i, pos_lower_limits(i));
  }
  space_->setBounds(bounds);
  // define a simple setup class
  og::SimpleSetup ss(space_);
  auto si = ss.getSpaceInformation();
  // setup the Validty Checker as a lambda function which returns a bool
  ss.setStateValidityChecker([&plant, &plant_context, &simulator,
                              &robot_dof_,
                              &FLAGS_collision_tolerance]
                              (const ob::State *state) {
    auto next_conf = OMPLStateToEigen(state, robot_dof_); // copy
    plant.SetPositions(&plant_context, next_conf);
    // Query port & object used to find out results from scene graph
    const auto &query_port = plant.get_geometry_query_input_port();
    if (!query_port.HasValue(plant_context)) {
      throw std::invalid_argument(
          "Cannot get a valid geometry::QueryObject. "
          "Either the plant geometry_query_input_port() is not properly "
          "connected to the SceneGraph's output port, or the plant_context is "
          "incorrect. Please refer to AddMultibodyPlantSceneGraph on "
          "connecting MultibodyPlant to SceneGraph.");
    }
    const auto &query_object =
        query_port.Eval<QueryObject<double>>(plant_context);

    // method is based on https://github.com/RobotLocomotion/drake/issues/11580
    std::vector<SignedDistancePair<double>> signed_distance_pairs =
                      query_object.ComputeSignedDistancePairwiseClosestPoints(FLAGS_max_distance);

    bool is_collision_free = true; // return value
    for (const auto &signed_distance_pair : signed_distance_pairs) {
      // stop once any collision is detected:
      if (signed_distance_pair.distance < static_cast<size_t>(FLAGS_collision_tolerance)) {
        // uncomment to display info on the colliding object; warning, very
        // verbose const auto& inspector = query_object.inspector(); const auto&
        // name_A = inspector.GetName(signed_distance_pair.id_A); const auto&
        // name_B = inspector.GetName(signed_distance_pair.id_B);
        // drake::log()->info("{} <--> {} is: {}", name_A, name_B,
        // signed_distance_pair.distance);
        is_collision_free = false;
        return is_collision_free;
      }
    }
    return is_collision_free;
  });

  // further setup the OMPL planning specifications
  ss.getSpaceInformation()->setStateValidityCheckingResolution(FLAGS_discretization);

  // set the desired planner
  ss.setPlanner(std::make_shared<og::RRTConnect>(si)); // InformedRRTstar
  auto t_finished_setup = hr_clock::now();

  ompl::base::ScopedState<> start(space_);
  start = eigen_to_vec(start_conf); // dru::e_to_v(start_conf);
  ompl::base::ScopedState<> goal(space_);
  goal = eigen_to_vec(goal_conf);
  ss.setStartAndGoalStates(start, goal);
  // this call is optional, but we put it in to get more output information
  ss.setup();
  ss.print();

  std::vector<Eigen::VectorXd> trajectory;
  if (ss.solve(FLAGS_solve_time) == ob::PlannerStatus::EXACT_SOLUTION) {
    ss.simplifySolution(FLAGS_simplify_time);
    og::PathGeometric path = ss.getSolutionPath();
    // determine duration forthe problem setup and solution
    auto t_finished_planning = hr_clock::now();
    auto delta_time_setup =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_finished_setup -
                                                              t_begin)
            .count();
    auto delta_time_planning =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            t_finished_planning - t_finished_setup)
            .count();
    drake::log()->info(
        "Setup {} sec. and Planning {} sec. complete in total time: {} "
        "seconds. Found path with length: {} and num_states: {}",
        delta_time_setup / 1000., delta_time_planning / 1000.,
        delta_time_setup / 1000. + delta_time_planning / 1000., path.length(),
        path.getStates().size());
    // Return solution from OMPL problem definition and convert to Eigen type
    path.interpolate(FLAGS_num_interpolated_states);

    std::vector<ob::State *> path_states = path.getStates();
    for (auto &ompl_state : path_states) {
      trajectory.push_back(OMPLStateToEigen(ompl_state, robot_dof_));
    }
  } else {
    return 1;
  }

  size_t num_cycles{0};
  while (num_cycles < static_cast<size_t>(FLAGS_cycles_to_display)) {
    for (size_t ii = 0; ii < 2; ii++) {
      for (const auto &conf : trajectory) {
        plant.SetPositions(&plant_context, robot_model_idx_, conf);
        diagram_->ForcedPublish(diagram_context);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(FLAGS_pause_msec));
      }
      std::reverse(std::begin(trajectory), std::end(trajectory));
    }
    num_cycles++;
  }

  std::cerr << "Planning Complete!" << std::endl;
  return 0;
}