ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
ros2 run base_path_generator path_checker_node --ros-args -p planner_id:=SmacHybrid
ros2 run base_path_generator base_path_generator_node


use GridBased for Navfn
use SmacHybrid for Smac Planner Hybrid A*
