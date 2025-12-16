#  DWA-RRT* Hybrid Path Planning (MATLAB)

## 1. Project Overview

This repository contains a MATLAB implementation of a **Hybrid Path Planner** for mobile robot navigation and obstacle avoidance. The architecture combines a global path-finding algorithm (**RRT*** ) with a local motion planning algorithm (**DWA**) to ensure both global optimality and safe, real-time reactive motion.

* **Global Planner:** Uses the **RRT*** (Rapidly-exploring Random Tree Star) algorithm to find an optimal (shortest) path from start to goal, respecting **static obstacles** (walls).
* **Local Planner:** Uses the **Dynamic Window Approach (DWA)** to follow the global path while dynamically avoiding **static and transient obstacles** by respecting the robot's kinematic and dynamic constraints.

The main entry point of the simulation is:

```
dwa_rrt_star_stable()
```

---

## 2. Algorithm Architecture: The Hybrid Approach

This implementation follows a **decoupled global–local planning strategy**:

* **Global Path Planning**
  Generates an optimal, globally collision-free path from start to goal using RRT*.

* **Path Following Logic**
  Selects a lookahead target point along the global path that the local planner should track.

* **Local Motion Control**
  Computes real-time velocity commands ((v, \omega)) using DWA while ensuring safety and smooth motion.

Relevant functions in the code:

* Global Planner: `rrt_star_planner`
* Path Target Selection: `select_target_index`
* Local Planner: `dwa_control`

---

## 3. Global Planner: RRT* (`rrt_star_planner`)

The **RRT*** algorithm is selected for its:

* **Probabilistic completeness**
* **Asymptotic optimality**, producing near-shortest paths in complex environments

### Key Parameters

* `config.rrt_max_iter`
  Maximum number of iterations allowed to search for a valid path.

* `config.rrt_step_size`
  Maximum distance between a new node and its parent.

* `config.rrt_goal_sample_rate`
  Probability of directly sampling the goal to bias tree growth.

* `config.safe_dist`
  Safety margin around static obstacles to ensure collision-free paths.

### Path Optimization (Rewiring)

RRT* performs **rewiring** whenever a shorter path to an existing node is found. This continuously improves the quality of the global path and ensures near-optimal performance.

---

## 4. Local Planner: Dynamic Window Approach (DWA)

The **Dynamic Window Approach** runs inside the main simulation loop and generates safe velocity commands based on:

* Robot dynamics
* Current state
* Obstacle proximity
* Global path target

### 4.1 Dynamic Window Calculation (`calc_dynamic_window`)

The **Dynamic Window (DW)** represents the set of all feasible velocities:

* **Kinematic Window (Vₛ):**
  Constrained by robot limits such as `max_speed`, `min_speed`, and `max_yaw_rate`.

* **Dynamic Window (V_d):**
  Constrained by acceleration and deceleration limits (`max_accel`, `max_yaw_accel`) over the time step `dt`.

This guarantees that all sampled velocity commands are physically achievable.

---

### 4.2 Trajectory Evaluation and Objective Function (`calc_control_and_trajectory`)

For each candidate velocity ((v, \omega)) in the dynamic window, a trajectory is predicted and evaluated using a cost-based objective function.

The selected trajectory minimizes the total cost, which consists of:

#### Obstacle Cost (`calc_obstacle_cost`)

* Inverse of the minimum distance to obstacles along the predicted trajectory.
* Set to **infinity** if a collision is detected within `config.robot_radius`.

Relevant parameters:

* `config.obstacle_cost_gain`
* `config.robot_radius`

#### Goal Cost

* Distance between the final predicted position and the current global target point.

Relevant parameter:

* `config.to_goal_cost_gain`

#### Speed Cost

* Encourages faster motion to reduce travel time.
* Computed as:

```
max_speed - v
```

Relevant parameter:

* `config.speed_cost_gain`

---

## 5. Running the Simulation

### 5.1 Requirements

* MATLAB (R2018a or newer recommended)

### 5.2 Execution Steps

1. Clone the repository and open MATLAB.
2. Open the file:

```
dwa_rrt_star_stable.m
```

3. Run the simulation:

```matlab
dwa_rrt_star_stable
```

### 5.3 Interactive Setup

* **Step 1:** Left-click to select the **START** position, then right-click to select the **GOAL** position.
* **Step 2:** The computed **RRT*** global path is displayed in **green**.
* **Step 3:** Left-click to add **dynamic obstacles** (shown as red squares).
* **Step 4:** Right-click to begin the DWA-based navigation simulation.

---

## 6. Key Configuration Parameters

The system behavior and performance are strongly influenced by the parameters defined in the `config` structure:

* `config.predict_time`
  Prediction horizon for each DWA trajectory.

* `config.dt`
  Time step for state integration and control updates.

* `config.to_goal_cost_gain`
  Weight for goal alignment (lower values emphasize obstacle avoidance).

* `config.obstacle_cost_gain`
  Weight for obstacle clearance (higher values increase safety).

* `config.robot_radius`
  Physical radius of the robot.

* `config.safe_dist`
  Safety margin for static obstacles used by the RRT* planner.

---

## 7. Code Structure

The implementation is modular and organized as follows:

* **Main Logic**
  `dwa_rrt_star_stable`

* **RRT* Planner**
  `rrt_star_planner`, `check_collision`, `interpolate_path`

* **DWA Core**
  `dwa_control`, `calc_dynamic_window`, `calc_control_and_trajectory`, `predict_trajectory`, `calc_obstacle_cost`

* **Helper Functions**
  `motion_model`, `draw_robot`, and visualization utilities

---

This hybrid planning framework demonstrates a robust and practical solution for autonomous mobile robot navigation in cluttered and dynamic environments.
