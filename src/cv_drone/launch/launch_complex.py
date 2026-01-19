import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import xacro
import csv

from typing import List

USE_SIM_TIME = LaunchConfiguration("use_sim_time", default="true")
USE_GUI = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"], description="Whether to execute gzclient")
PKG_GAZEBO_ROS = get_package_share_directory('gazebo_ros')
XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
XACRO_FILE_PATH = os.path.join(get_package_share_directory("sjtu_drone_description"),"urdf", XACRO_FILE_NAME)
WORLD_DIR = "/workdir/worlds"
WORLD_FILE = os.path.join(WORLD_DIR, "complex_test.world")

CSV_PATH = "/workdir/data/drones_one.csv"


def read_drones_csv(csv_file: str)->tuple[List[str], dict[str,str]]:
    """
    description:
        - reads a CSV file with columns 'drone_id', 'x_pos', 'y_pos' and returns a list of drone IDs and a dictionary of their positions.
    inputs:
        - `csv_file` filepath continaing namespace ('drone_id') and start position (x_pos and y_pos)
    outputs:
        - `drone_ids` is List[str] containing namespaces
        - `drone_positions` is dict[str, str], where keys are namespace and values are robots' start positions represented as str
            - example: drone_positions["r_0"] = "1.10 2.20 3.30" corresponds to a drone with namespace of "r_0" starting at position (1.1, 2.2, 3.3)
    """
    drone_ids = []
    drone_positions = {}

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            # print(row)
            # print(row.keys())
            drone_id = row['drone_id']
            x_pos, y_pos = row['x_pos'], row['y_pos']

            drone_ids.append(drone_id)
            drone_positions[drone_id] = x_pos +" " + y_pos + " " + "1.0"

    return drone_ids, drone_positions

def spawn_drone_description(ns: str, init_pose: str, use_sim_time: bool)->List[Node]:
    """
    description:
        - returns list of nodes to spawn the drone with the provided namespace
    inputs:
        - `ns` is namespace of the drone
        - `init_pose` is initial pose represented as a str - example "1.10 2.20 3.30" corresponds to (1.1, 2.2, 3.3)
    outputs:
        - `result` is list of nodes to launch
    """

    r_n_doc = xacro.process_file(XACRO_FILE_PATH, mappings = {"drone_id":ns})
    r_n_desc = r_n_doc.toprettyxml(indent='  ')

    result = [
            Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{'frame_prefix': ns +'/','use_sim_time': use_sim_time, 'robot_description': r_n_desc}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=ns+"_" +'joint_state_publisher',
            namespace=ns,
            output='screen',
        ),
        
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[r_n_desc, ns, init_pose],
            output="screen"
        )
    ]
    
    return result

R_NS, init_poses = read_drones_csv(CSV_PATH)

def multi_drone_description(R_NS: List[str], init_poses: dict[str])->List[Node]:
    """
    description:
        - takes the output from `read_drones_csv` and generates nodes to multiple drones (1 or more)
    inputs:
        - `R_NS` list of namespaces for multiple drones
        - `init_poses` dictionary mapping start poses to robots by their namespace
    outputs:
        - `result` is list of nodes to launch
    """
    result = []
    for ns in R_NS:
        drone_spawn_instructions = spawn_drone_description(ns, init_poses[ns], USE_SIM_TIME)
        result = result + drone_spawn_instructions
    return result

def drone_bringup(ns: str)->List[Node]:
    """
    description:
        - returns a list of nodes for drone's behaviour to launch with the provided namespace
    inputs:
        - `ns` is namespace (example robot1) - useful for launching multiple robots
    outputs:
        - nodes to launch for a specific robot (namesapce)
    """

    result = [
        Node(
            package="cv_drone",
            executable="teleop_node",
            namespace=ns,
            output="screen",
            prefix="xterm -e"
        ),
        Node(
            package="cv_drone",
            executable="pose_reporter_node",
            namespace=ns,
        ),
        Node(
            package="cv_drone",
            executable="vision_node",
            namespace=ns,
        ),
        Node(
            package="cv_drone",
            executable="plotter_node",
            namespace=ns
        )
    ]

    return result

def multi_drone_bringup(R_NS)->List[Node]:
    """
    description:
        - executes `drone_bringup` for each drone in the list of namespaces
    inputs:
        - `R_NS` list of namespaces for multiple drones
    outputs:
        - `result` is list of nodes to launch
    """

    result = []
    for ns in R_NS:
        drone_brinup_nodes = drone_bringup(ns)
        result = result + drone_brinup_nodes
    return result

def launch_gzclient(context, *args, **kwargs):
    """
    description:
        - launches gazebo client for robot simulation
    """
    if context.launch_configurations.get('use_gui') == 'true':
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': 'true'}.items()
        )]
    return []

def generate_launch_description():
    """
    description:
        - launches nodes for robot simulation and robot actions/behaviour
        - required in a ros2 launch file
    """

    LD =[
        USE_GUI,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': WORLD_FILE,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient)
    ]

    LD = LD + multi_drone_description(R_NS, init_poses)
    LD = LD + multi_drone_bringup(R_NS)
    return LaunchDescription(LD)