# haptiquad_ros2
## Description
`haptiquad_ros2` is a ROS2 humble wrapper for the [`haptiquad`](https://github.com/mlisi1/haptiquad) library. The main purpose of this wrapper is to interface the library with different simulations (Gazebo, Mujoco and rosbags for the moment) and be able to calculate residuals and estimate ground reaction forces as well as the floating base wrench. This repo offers the following packages:
+ `haptiquad` - the library implementing all the core code
+ `haptiquad_ros2` - contains the wrappers for the different simulations
+ `haptiquad_msgs` - messages for residuals, force, and errors
+ `haptiquad_plot` - implements visualization tools for estimated data as well as some debugging tools

## Installation:
To install the package use the following commands:
```bash
git clone git@github.com:mlisi1/haptiquad_ros2.git
cd haptiquad_ros2/
git submodule update --init --recursive --remote
cd ..
bash haptiquad_ros2/build.sh
source install/setup.bash
```
The build command will succeed only if the required packages for at least one of the simulation wrappers are present; otherwise, it will fail.

## Usage
For examples of complete simulations that exploit `haptiquad_ros2` see: [`haptiquad_mujoco`](https://github.com/mlisi1/haptiquad_mujoco/tree/main), [`haptiquad_gazebo`](https://github.com/mlisi1/haptiquad_gazebo), [`haptiquad_rosbag`](https://github.com/mlisi1/haptiquad_rosbag).

Every wrapper has some topics in common under the `/haptiquad_ros2` namespace:
```
/gains              # Subscribed - allows to change the observer's gains during runtime
/friction           # Subscriber - allows to change friction parameters during runtime
/residuals          # Published - internal and external residuals are published here
/residual_errors    # Published - (optionally) the residual error (between estimated residuals and calculated residuals from the real force measurements)
/estimated_forces   # Published - published the estimated foces and base wrench
```

Beside these topics, there are subscription topics to joint states, floating base spatial velocity and orientation, which are needed for the estimation; these depend on the type of simulation.
In every wrapper where the data is available, if the parameter `calculate_residual_error` is set to `true`, it will switch to a callback that will also listen to the true contact forces and base wrench obtained for the simulation, calculating the error between the estimated residuals and the true residuals.

## Visualization
To effctively visualize residuals and forces, `haptiquad_plot` implements custom GUI that allow the correct plot of both, as well as relevant error data, and the possibility to save the plots.
#### Residual Plotter
ResidualPlotter allows to see both the external and internal residual, with the latter one divided for the robot legs. The node has implemented callbacks for the same simulations as the wrappers.
![ResidualPlotter](res/residual_plotter.png)
It can be tweaked by modifying the following parameters:
```yaml
x_lim:  10.0            # seconds interval to plot; it is equivalent to the Autoscroll button and spinbox
autoscale: True         # toggles plots Y axis autoscaling; it is equivalent to the Autoscale button
meoty_limit: 1000       # max length of data saved
listening:  True        # whether to start listening to topics when started or not
legs_prefix : []        # list of strings containing the prefixes identifying the robot legs
```
Beside plotting residuals, it is possible to change the observer's gains during runtime and switch to the visualization of reisudal error (if available).
#### ForcePlotter
ForcePlotter is a node that allows to plot estimated forces, as well as error data calculated with the real force measurements. Similarly to the previous tool, it contains the correct callbacks to interface with the implemented simulations.
![ForcePlotter](res/force_plotter.png)
It is possible to visualize:
+ Estimated forces only (Estimate)
+ Real measurements from simulation (GT)
+ Error between real and estimated forces (Error)
+ RMSE
+ Norm error, calculated excluding the torque components (Norm)
+ Estimated and real forces in the same plot (GT+EST)

The tool's parameters are the same as ResidualPlotter's, with the addition of `foot_suffix`, which is a string specifying the suffix in the feet frames' name. For example, ANYmal C has feet frames named "LF_FOOT", "RF_FOOT", and so on, thus the parameter shall be "FOOT".

## Adding new wrappers:
`haptiquad_ros2` allows the creation of new wrappers without much code modifications. It contains the WrapperBase class (`haptiquad_ros2/src/wrapper_base.hpp`), which implements all the common methods and attributes. The only additions needed to add new wrappers is the creation of a new class inheriting from WrapperBase, which adds the necessary subscription to the needed data topics (usually, message_filters are used, as it's unlikely to have a single topic for everything), and its relative callback.


