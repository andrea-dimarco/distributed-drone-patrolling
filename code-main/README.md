# IoT-Project v1.1.2

<p align="center">
  <img src="https://fede3751.github.io/IoT_lectures/imgs/project/project_splashart.png">
</p>

Official repository for the IoT project.

You are required to perform a patrolling task on the given target points.
Each target has a certain amount of time in which it has to be patrolled.
If a drone does not patrol a target in the given threshold time, the target is considered "expired".

You are given a simulation configuration, with a variable number of drones, targets, targets' positions and their threshold times.<br>
The simulation additionally gives you the weight of each metric presented in class.<br>
Refer to this file if you have missed that: <a href="https://fede3751.github.io/IoT_lectures/misc_files/project_description.pdf">project_description.pdf</a>.<br><br>

Your solution will have to provide a patrolling algorithm which takes into account the weights of each performance metric. All these metric will be considered with respect to their weight to compute a final evaluation.

## Grader

<p align="center">
  <img src="https://fede3751.github.io/IoT_lectures/imgs/project/grader_display.png">
</p>

The grader will display the values of your simulation live, while keeping track of the values of each performance metric.<br>
For each target, you can check at any given time its threshold time and its AoI function.<br><br>
A good solution should be able to keep the Cumulative AoI and Violation as low as possible, while trying to keep the fairness as close as possible to a value of 1.<br><br>
The importance of which value to prioritize should be decided based on the weights given for each of the three evaluation metrics.
In the general case, with equal weights among the three metrics, you should always strive to have the least amount of violation, then to minimize the cumulative AoI, and only after that to have a fair solution.<br><br>

The score is calculated as follows:

<p align="center">
  <img src="https://fede3751.github.io/IoT_lectures/imgs/project/score_calculation.png">
</p>


## Launching the Simulation

Just clone this repository:

```
git clone https://github.com/Fede3751/IoT-Project.git
```

Additionally, for the grader to work, you may need to manually install the <a href="https://github.com/hoffstadt/DearPyGui">DearPyGui</a> library, which is not found by rosdep by default:
```
pip install dearpygui
```


Build the workspace

```
colcon build
```

And then start the simulation

```
./start
```

If you want to load a custom simulation file, you can just add it to the arguments of the <code>start</code> executable:

```
./start ./sim_files/your_simulation.sim
```

This executable will automatically compile your solution package, source the environment, and launch the simulation.<br><br>

## Package Organization

The workspace is composed of multiple packages:<br><br>
**iot_project**<br>
  &emsp; Wraps all the other packages together and exposes one launch file to launch the whole simulation<br>
**iot_project_manager**<br>
  &emsp; Responsable of keeping track of drones and target positions and update simulation values correctly<br>
**iot_project_grader**<br>
  &emsp; Used to visualize the current score, by reading values outputted by the tester<br>
**iot_project_animator**<br>
  &emsp; C++ package currently used only to change color of the spheres in the simulation<br>
**iot_project_interfaces**<br>
  &emsp; Stores all the interfaces used in this workspace<br>
**iot_project_solution_src**<br>
  &emsp; Package for storing your solution to the project<br>
**iot_project_solution_interfaces**<br>
  &emsp; Package for storing your solution interfaces to the project<br><br>

## Simulation Instance

A simulation instance can be fully paremtrized by providing a .sim file to the launch file. Current start file automatically selects <code>./sim_files/simple_simulation.sim</code>, which looks like this:<br>

```
# simulation name can be defined like this
simulation_name:Simple Simulation

# specifies the number of drones in the simulation
no_drones:6

# specifies simulation time in seconds
simulation_time:300

# specifies targets positions
target_positions:(-5,-5,3);(-5,2,7);(7,7,7);(3,-5,6);(5,5,7)

# specifies threshold values for every target
threshold_values:25,30,25,25,30

# specifies aoi weight
aoi_weight:1.0

# specifies violation weight
violation_weight:1.0

# specifies the fairness' weight
fairness_weight:0.5

# specifies wind vector
wind_vector:(0,0,0)

# specifies wind strength
wind_strength:0
```

Simulation instances will be provided every week for you to test your solution against new challenges.


## Solution Submission

Your solution should go in:

```
/src/iot_project_solution_src
/src/iot_project_solution_interfaces
```

Note that files outside of these packages won't be considered for your submission.<br><br>


A baseline solution which schedule a random patrolling, with not-so-good movements between targets is given as a starting point. This solution is composed mainly of these two files:<br>
- <code>drone_controller.py</code> which is responsable of moving the drones between a set of points. It should be modified to improve the actual physical, movement of drones.<br>
- <code>task_assigner.py</code> is instead used to schedule a patrol algorithm of your choice, which submits patrol action to every drone in the simulation. Current solution, as already said, just schedules a random patrolling each time the previous one is completed.<br>

You are free to create and spawn new nodes within your package if you see fit for your solution, but editing the already given ones is encouraged.<br><br>

### Good luck!
