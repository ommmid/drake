# My Note

Sampling method example
PRM
RRT
`/home/omid/manipulation/manipulation/exercises/trajectories/rrt_planner/rrt_planning.py`
example of it is at:
`/home/omid/manipulation/book/trajectories/exercises/rrt_planning.ipynb`

TrajOpt example
directiontranscription
directioncollocation
trajectoryoptimiztion

what can be done offline to save time in optimization problem?

example for
constraint in cartesian space

example
Redundnat Manipulator: seconday objective

## inspecting kinematic tree
`manipulation/book/pick/kinematic_tree.ipynb`

## forward kinematics
`manipulation/book/pick/forward_kinematics.ipynb`

## Jacobian - geometric
`manipulation/book/pick/jacobian.ipynb`

## Jacobian pseudoinverse
`manipulation/book/pick/pseudoinverse.ipynb`

The pseudo-inverse is a beautiful mathematical concept. 
- When the is square and full-rank, the pseudo-inverse returns the true inverse of the system. 
- When there are many solutions (here many joint velocities that accomplish the same end-effector spatial velocity), then it returns the minimum-norm solution (the joint velocities that produce the desired spatial velocity and are closest to zero in the least-squares sense). 
- When there is no exact solution, it returns the joint velocities that produce an spatial velocity which is as close to the desired end-effector velocity as possible, again in the least-squares sense. So good!

Singularity
A better metric, is to watch the smallest singular value; as this approaches zero, the norm of the pseudo-inverse will approach infinity. It is getting close to singularity.
The robot can go through singularity perfectly fine from joint space point of view, but the map from ee
to joint tells you that it can not. We are using first order derivative (jacobian), there is still
second order derivative that exists, the acceleration.

## IK
solve IK as a constrained optimization. Instead of using velocity and jacobian, we solve for solution that is close to the current configuration with forward kinematics being the constraint
`https://manipulation.csail.mit.edu/trajectories.html#:~:text=IK%20as%20constrained%20optimization`
In the ipynb, we see the jumps to the next position for that. No command sent to the joints controller, it is just
changing the configureation of the robot. In this Ik optimization, we might find other solutions, of the IK but in diff IK (the jacobian one), we always find one that is closest to the current one. 

SNOPT is used to solve this nonconvex optimization
`manipulation/book/trajectories/interactive_ik.ipynb`

Now having all the solutions might be problem in the IK approach, because the arm might jump between 
solutions in one time steps. Look at the second example. sNOPT solves the nonconvex problem and finds solution
that are not in collision but it might not be smooth to transtion to them. In that case we do not want to use 
the output of the IK approach straight to the controller

## Motion planning - optimization

trajectory optimiztion:
    - direct transcription. discritizing the optimization problem. Usually min effort for cost
    and bunch of constraints which includes system dynamics
    not good when we need contact or collision (things from GraphScene)

    - shooting: its like direct transcription but we remove the state from decision variables, we 
    can have single shooting or multiple shooting

    - direct collocation. like direct transcription but uses cubic spline to reprent the path, so we can define less control points
    resuling in less variables. 
    `underactuated/book/trajopt/double_integrator.ipynb`
    `underactuated/book/trajopt/dircol.ipynb`

    - kinematics trajectory optimization:
    This is an extension of prvious IK approach but we are going to solve for a handful of points not one point. This is a kinda of DirectCollocation but without dynamics
    `manipulation/book/trajectories/kinematic_trajectory_optimization.ipynb`
    Drake uses B-spline which has the convex hull property to ensure that limits on the joint positions and any of its **derivatives** are satisfied. Note this is used in the optimiztion process not after calculating the path. Kinematics here does not mean that we do not set time derivative constraint but it means we do not have the dynanmics equations in the equality constraint like DirecCallocaiton or DirectTranscription have. Also, decision variables includes only q compare that with [q, qdot] and u (input) in dynamics optimization

Collocation is an idea about gradients of a continuous trajectory. It is only a relevant idea for continuous-time systems.

Direct transcription is well-defined for both discrete- and continuous- time. 

Direct transcription and collocation are not suitable for contact and collision check

The default solver is the SQP-solver, SNOPT.

some good examples:
https://github.com/vincekurtz/drake_ddp/blob/b4b22a55448121153f992cae453236f7f5891b23/acrobot.py#L177 

Some other trajectory optimization methods that got attention:         
**CHOMP**: projected gradient descent which satifsying collision            
**STOMP**: gradient free that relies on generting noisy trajectories            
**TrajOpt**: it does a Sequentation Quadratic Programming. We need to have the objective function and the constraint ready to go for a QP at each sequence. Objective function is min distance. Two things:
- they use trust region to find the right region for approximation to QP            
- convert constraints into penalty terms in cost function. To do that, they use $l_1$ penalty (like absolute value). This penalty is called exact penatly because, with a high enough weight, it will converge to zero like the constraint is being satisifed completely. This is not true for $l_2$ penalty where the norm 2 is used. Equality constratin become like absolute value penalty and inequality constraint are basically Relu function $max(x,0)$ 
- to smooth these $l_1$ terms, they use slack varialbes         
- They linearize the signe distance function, so we can easily add them to the cost function of the QP
- They use Bullet because it can find convex-convext collision check (GJK method). FCL finds all the contact points but Bullet find the deepest penetration.
- You can add constraint in joint space or Cartesian space. Also, you can add each of them for different segment of the trajectory.     

distance fields vs convex-convex collision check:       
distane feild is a pre-computed function on a voxel  grid that specifies the distance to the nearest collision. The advantage is the collision check is constatn time. whereas convex-convex colision detection takes two shapes and computes minimal translation to get them out of collision or make them collide. The distance field is merely on points or sphere on the robot while convex-convex detection is the convext shape of each link of the robot. CHOMP uses distance field and trajopt uses convex-convex collision check        

SQP vs project gradient descent:        
CHOMP uses projected gradient descent where at each step the joint values are projected to the constrained space to make sure constraint are satsiftied at each step. This is cheaper than solving QP (like trajopt does) but gradient descent does not take into account the second derivative

## Motion planning - sampling based
PRM (multiple enquery): roadmap construction, offline. graph search (online)
RRT (singel enquery)

There was an effort in Drake to use OMPL but it did not go anywhere

## Trajectory Generation
Time parameterize a path from (motion planning) so we can execcute that plan subject to velocity, 
acceleration and torque-limit constraint

**Trapezoidal** velocity and trpezoidal acceleation are two common methods.

<!-- ![left: trapezoidal velocity trajectory, right: trapezoidal acceleration trajecotyr from Ruckig](./share/trapez.png) -->
<figure>
<img src="./share/trapez.png" width=800 >
<figcaption>left: trapezoidal velocity trajectory, right: trapezoidal acceleration trajecotyr from Ruckig</figcaption>
</figure>

We also could use **polynomials** of 3rd order or 5th roder. They are easy  to fit differentiable and continous. _Minimum jerk_ and _minimum snap_ trajectories are the polynomial  trajectories that minimizes jerk or snap.   
in trapezoidal profiles approach, we had hard constraints on following those trapezoids but in polynomial we made it loose. There is not guarantee that joints limits are satisfied between control points of the polynomials.      

**B-spline** is the approach to go becuse it gaurantees that the intermidiates poitns between control points stay in the convex hull and their derivaties do not overshoot. The tradeoff is that B-spline may not pass through all the control points (first and the last is guaranteed though)

**Time-optimal Trajectory Generation** (TOTG) is another method which attempts to alternate between max acceleration and deceleration (bang-bang)

In drake you could use `PathParameterizedTrajectory` to do these stuff


## Optimization
in drake language, mathematical programs is the same is calling optimization methods
```
prog = MathematicalProgram()
x = prog.NewContinuousVariables(2)
prog.AddConstraint(x[0]+x[1] == 1)
prog.AddConstraint(x[0] <= x[1])
prog.AddCost(x[0] ** 2 + x[1] ** 2)
result = Solve(prog)
```
## SQP
sequential quadratic programming:
Uses a lagrange multiplier to combine the equality and inequality constraints into cost function as penalty t erms. SQP alrogithm uses Newton's method, the one for optimization method not root finding, thus second order derivative is needed. But that means they can convert the problem into quadratic cost function locally.

Wikipedia has really good clear explanation: https://optimization.cbe.cornell.edu/index.php?title=Sequential_quadratic_programming

## BAZEL
bazel fetch @ompl//:ompl-1.5

## Polytope
when a polytope has dimension n=3 we say it is a polygedron     
when a polytope has dimension n=2 we say it is a plygon     
polytope in general has dimension n     

<img src="./share/poly.png" width=400>