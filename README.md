# Follow the Gap on F1Tenth: Catalunya track

Reactive Follow the Gap controller for the F1Tenth simulator with ROS 2 Humble. I developed it for the First Partial Project of Autonomous Vehicles at ESPOL. This README covers both parts. Part 1 is the empty track. Part 2 adds obstacles and a second robot that has to be overtaken. In both parts the car drives the Catalunya track using only the LIDAR, with no map and no saved trajectory.

Author: David Alejandro Acosta Lara

## Videos

- Part 1, 10 laps on the empty track: https://youtu.be/JLJOk-7RoX0
- Part 2, 10 laps with obstacles and a second robot: https://youtu.be/Tq_DhnBHp64

# Part 1: Follow the Gap on the empty track

## Result

- 10 consecutive laps completed, without any collision.
- Lap time of around 51 s.
- Evidence video: https://youtu.be/JLJOk-7RoX0

The video shows the 10 laps with the lap counter, the stopwatch and the best time. The controller draws them in RViz and also prints them in the terminal.

## 1. Approach

The car has a LIDAR of 1080 rays that covers 270 degrees, from -135 to +135, with 0.25 degrees between one ray and the next. Each ray says how far the obstacle is in that direction.

The controller is reactive. On every scan it decides only from what it sees at that moment, with no memory and no map. That is why it responds fast and does not depend on the track, but it also does not know which way the race goes. If it crashes, it can end up facing backwards and leave in the opposite direction.

Follow the Gap means following the gap. The idea is to look for the widest free corridor in front of the car and aim at it. I built it in stages, and each one fixed a problem I saw in the previous one:

- Stage 1. Go straight at a fixed speed. The car crashes at the end of the straight.
- Stage 2. Go toward the farthest ray. It crashes in the first curve, and the rays tied at 10 m confuse it.
- Stage 3. Bubble and longest gap. It crashes in the same curve, because the bubble only protects the closest obstacle and the dangerous one was another.
- Stage 4. Widen the corners. It completes the lap, but rubbing the walls.
- Stage 5. Center the target inside the gap. It separates from the walls.
- Stage 6. Variable speed with a braking rule. The time goes from more than 195 s down to about 51 s.
- Stage 7. Lap counter, stopwatch and text in RViz.

### What the controller does on every scan

- Preprocess. The inf and NaN values are replaced by 10 m, everything is clipped to 10 m and the rays that look backwards are dropped, keeping the ones from -90 to +90 degrees. The angle of each ray comes from angle_min plus the ray number times angle_increment.
- Widen the corners. If two neighboring rays differ by more than 0.4 m, there is a corner. The far ray passes grazing it, but the car has width and does not fit there. On the far side I replace n rays with the short distance of the corner. First I compute half the car plus the safety margin, and n is the angle that this distance takes seen from the corner, divided by angle_increment and rounded up. This stage is the one that made the car complete the lap without crashing.
- Bubble. I look for the shortest ray and set to 0 the rays that fall within 0.45 m around it.
- Gap. I look for the longest sequence of consecutive rays with a distance greater than 1.5 m.
- Target. Inside the gap I take the farthest point, and if there are several tied, the one in the middle. Then I mix it with the center of the gap, 65 percent farthest point and 35 percent center. Without this mix the car aimed at the edge of the gap and stuck to the walls.
- Command. The steering is the angle of the target, limited to 0.4189 rad. The speed comes from two rules and the lower one is used.

### The two speed rules

- Steering rule. With straight steering it goes at v_max, and with a very turned wheel it slows down to v_min. In between it is shared in proportion. I compute the proportion as the smaller value between 1 and the steering angle divided by steer_for_vmin, and the speed is v_max minus that proportion times the difference between v_max and v_min.
- Front distance rule. It comes from the formula v squared equals v0 squared plus 2 times a times d. I take the minimum distance in a window of 8 degrees on each side of the front and subtract front_margin, without letting it become negative. The maximum safe speed is the square root of v_min squared plus 2 times brake_decel times that free distance.

With the two rules the car speeds up on the straights, brakes before the curves and takes them slowly.

### Final parameters

- max_range: 10.0 m. Maximum distance that is considered.
- car_width: 0.31 m. Width of the car.
- safety_margin: 0.15 m. Extra margin on each side.
- disparity_threshold: 0.4 m. Jump between rays that is considered a corner.
- bubble_radius: 0.45 m. Radius of the bubble.
- gap_threshold: 1.5 m. Minimum distance for a ray to count as free.
- center_weight: 0.35. Weight of the gap center in the target.
- v_max: 9.5 m/s. Speed with straight steering.
- v_min: 5.0 m/s. Speed in the tightest curves.
- steer_for_vmin: 0.30 rad. Steering angle at which v_min is used.
- brake_decel: 5.5 m/s squared. Braking that I assume the car can do.
- front_margin: 2.0 m. Distance that I want to keep in front.
- front_cone_deg: 8 degrees. Half the width of the front window.

These values came from tests in the simulator. I changed one parameter per test and repeated the 10 laps until none of them crashed.

## 2. Lap counter and stopwatch

The node also subscribes to /ego_racecar/odom to know where the car is.

- On the first message it saves the starting position and where the car faces. That is the start line, a line perpendicular to that direction that passes through that point.
- On every message it computes how far ahead or behind the line the car is. It is the dot product between the displacement from the starting position and the starting direction.
- It counts a lap when that value goes from negative to positive, which means the car crosses forward. It also requires being less than 3 m from the line and having driven more than 30 m since the previous lap, to avoid counting false laps.
- When it counts a lap it measures the time with the node clock, updates the best time and restarts the stopwatch.

The state is shown in two ways:

- In the terminal, the log prints one line for every completed lap and, in the continuous log, the number of laps and the best time.
- In RViz, the node publishes a text marker on /lap_info, of type MarkerArray, attached to the car. It shows the laps, the time of the current lap, the last lap and the best one.

## 3. Code structure

```
src/controllers/
    setup.py                       registers the mi_gap executable
    controllers/
        mi_gap_node.py             the whole controller
src/f1tenth_gym_ros/
    config/sim.yaml                map and starting pose
    maps/Catalunya_map.png         image of the track
    maps/Catalunya_map.yaml        map data
```

The whole controller is in the MiGap class of mi_gap_node.py:

- preprocess. Cleans the scan and keeps the rays from -90 to +90 degrees.
- extend_disparities. Widens the corners.
- apply_bubble. Sets to 0 the rays near the closest point.
- find_max_gap. Finds the longest gap.
- find_best_point. Chooses the target, mixing the farthest point with the center of the gap.
- scan_cb. Runs on every scan, chains all the steps and publishes on /drive.
- odom_cb. Detects the crossing of the start line, counts the laps and measures the times.
- status_cb. Publishes the text of /lap_info for RViz.

Topics that it uses:

- It subscribes to /scan, of type LaserScan.
- It subscribes to /ego_racecar/odom, of type Odometry.
- It publishes on /drive, of type AckermannDriveStamped.
- It publishes on /lap_info, of type MarkerArray.

## 4. How to run it

You need ROS 2 Humble and the f1tenth_gym_ros simulator, installed as in the course tutorial. In my case I run them inside a Docker container with Ubuntu 22.04 and Humble. This repository already includes the Catalunya map.
VIDEO HOW TO RUN: https://youtu.be/4t9ZTQNw1KU 

### Prepare

- Clone the repository and build:

```bash
git clone https://github.com/Daalacos3005/f1tenth-follow-the-gap.git ~/F1Tenth-Repository
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

- Check the map. In src/f1tenth_gym_ros/config/sim.yaml, map_path must point to the real path of Catalunya_map on your computer and map_img_ext must be .png. The starting pose that I used is sx 0.0, sy 0.0 and stheta -2.14.

### Run

- In the first terminal, launch the simulator:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

- In a second terminal, run the controller:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers mi_gap
```

- To see the counter in RViz, use Add, then By topic, choose /lap_info and add the MarkerArray. The yellow text appears next to the car.
- To stop the controller, Ctrl+C. The car brakes before the node closes.

In the final version of the repository, sim.yaml is set up for Part 2. To run Part 1 again, set map_path to Catalunya_map and num_agent to 1.

## 5. Limitations

- Being reactive it has no memory, so after a crash it can leave in the opposite direction.
- With max_range at 10 m it does not see what is farther away. Raising it to 15 m could help to brake earlier, but I did not test it with the 10 full laps.
- The speeds are close to the limit. If the load of the computer or the latency changes, it might not complete the 10 laps without crashing.

# Part 2: obstacles and a second robot

## What Part 2 asks for

- Run 10 consecutive laps with no collision on Catalunya with at least 5 obstacles placed on the track.
- Add a second robot that starts 5 m ahead of the main car. The main car has to overtake it and complete 10 laps with no collision.
- Show a lap counter and a stopwatch for each lap while the car drives.

## Result

- 10 consecutive laps with no collision, with more obstacles than the 5 required and the second robot on the track.
- The best lap of Part 2 is 55.876 s.
- Evidence video: https://youtu.be/Tq_DhnBHp64

## What I did

- I kept the Part 1 controller exactly as it was. It is still mi_gap_node.py and it runs with ros2 run controllers mi_gap. It does not have the line that marks the path of each lap.
- I made a separate file for Part 2 called mi_gap_parte2_node.py. It runs with ros2 run controllers mi_gap_parte2.
- I made a file for the second robot called opp_gap_node.py. It runs with ros2 run controllers opp_gap.
- I painted the obstacles by hand on a copy of the track image. The original map is untouched.
- I changed sim.yaml so the simulator loads two cars and the new map.

### The Part 2 controller

It is the same Follow the Gap pipeline as Part 1. There is no overtaking logic. The second robot and the obstacles are treated like any other closest object, and the car goes around them through the gap. The main car overtakes simply because it is faster than the second robot.

Compared with Part 1, it has two changes:

- The front cone of the braking rule is wider, 10 degrees instead of 8. The car notices an obstacle or the other robot a little earlier, before it is directly in its way. The other parameters are the same as in Part 1.
- It draws a green line on the path of the car in RViz. The line is cleared every time the car crosses the start line, so it always shows the current lap. This line does not exist in the Part 1 controller.

The lap counter, the lap stopwatch and the text in RViz work the same as in Part 1.

### The second robot

opp_gap_node.py is a copy of the Part 1 controller with the lap counter removed. It listens to /opp_scan and publishes to /opp_drive. Its speed is limited to between 3 and 5 m/s, so the main car can catch it. It starts 5 m ahead of the main car on the same heading.

### The map with obstacles

- The files are Catalunya_obs.png and Catalunya_obs.yaml in src/f1tenth_gym_ros/maps.
- The obstacles are small black squares painted on the track.
- The image has to be 8 bit grayscale. If the image editor saves it in color, RViz shows the error Frame map does not exist and the simulator does not start. It can be fixed with this command, run from the repository folder:

```
python3 -c "from PIL import Image; p='src/f1tenth_gym_ros/maps/Catalunya_obs.png'; Image.open(p).convert('L').save(p)"
```

### Simulator configuration

In src/f1tenth_gym_ros/config/sim.yaml:

- num_agent is 2.
- map_path points to Catalunya_obs, without the file extension.
- The main car starts at 0.0, 0.0 with heading -2.14.
- The second robot starts at -2.70, -4.21 with the same heading, which is 5 m ahead of the main car.

Both /drive and /opp_drive must receive commands. If only one controller is running, neither car moves.

### Files of Part 2

```
src/controllers/
    setup.py
    controllers/
        mi_gap_node.py
        mi_gap_parte2_node.py
        opp_gap_node.py
src/f1tenth_gym_ros/
    config/sim.yaml
    maps/Catalunya_obs.png
    maps/Catalunya_obs.yaml
```

## How to run Part 2

The setup is the same as in Part 1: ROS 2 Humble, the simulator and the cloned and built repository. Start the container first. The prompt changes to root, which means you are inside. The next steps are all inside the container.

```
~/f1tenth_docker/run.sh
```

- Check sim.yaml. map_path must point to the real path of Catalunya_obs on your computer, map_img_ext must be .png and num_agent must be 2.
- Terminal 1, launch the simulator. RViz opens with the track, the obstacles and both cars.

```
cd /home/david/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

- Terminal 2, start the second robot. Open it with docker exec -it f1tenth bash.

```
cd /home/david/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers opp_gap
```

- Terminal 3, start the main car the same way.

```
cd /home/david/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers mi_gap_parte2
```

- To see the lap counter in RViz, use Add, then By topic, choose /lap_info and add the MarkerArray. The yellow text appears next to the car and the green line shows the current lap.
- To keep the camera on the main car, set the Target Frame of the view to ego_racecar/base_link.
- To stop, press Ctrl+C in each terminal. To repeat a run, restart the simulator too, so both cars go back to their starting poses.

## Common problems

- Package controllers not found means the source install/setup.bash line is missing in that terminal.
- No executable found means the workspace has to be rebuilt with colcon build after changing setup.py.
- RViz showing Frame map does not exist means the obstacle map is not in grayscale. Use the conversion command above.
- Neither car moves when only one controller is running in Part 2. Start both.

## Limitations of Part 2

- The controller has no memory. After a bad avoidance it can turn around and drive the wrong way, which would count as a failed run.
- The overtake depends on the speed difference between the two cars. If the second robot is made much faster or much slower, the result changes.

# Use of AI tools

I used Claude, from Anthropic, as an assistant during the project. It helped me diagnose errors, use GitHub and improve certain parts of the controller mi_gap_node.py where I was stuck.

# References

- Course repository: https://github.com/widegonz/F1Tenth-Repository
- F1Tenth Gym simulator: https://github.com/f1tenth/f1tenth_gym
