from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    eater_namespace = "eater_turtle"            # change eater_turtle namespace over here
    killer_namespace = "killer_turtle"          # change killer_turtle namespace over here

    launch_description = LaunchDescription()

    sampling_frequency = LaunchConfiguration("sampling_frequency")
    sampling_frequency_launch_arg = DeclareLaunchArgument(
        "sampling_frequency",
        default_value="100.0",
    )
    launch_description.add_action(sampling_frequency_launch_arg)

    # ---------------- Turtlesim Node ----------------
    turtlesim_node = Node(
        package="turtlesim_plus",
        namespace="",
        executable="turtlesim_plus_node.py",
        name="sim1",
    )
    launch_description.add_action(turtlesim_node)

    # ---------------- Eater Node ----------------
    eater_node = Node(
        package="lab3",
        namespace=eater_namespace,
        executable="eater.py",
        name="eater",
        parameters=[{"sampling_frequency": sampling_frequency}],
    )
    launch_description.add_action(eater_node)

    # ---------------- Killer Node ----------------
    killer_node = Node(
        package="lab3",
        namespace=killer_namespace,
        executable="killer.py",
        name="killer",
        parameters=[
            {"sampling_frequency": sampling_frequency},
            {"eater_name": eater_namespace},  # ให้ killer ไล่ชื่อ eater ที่ถูกต้อง
        ],
    )
    launch_description.add_action(killer_node)

    # ---------------- Remove default turtle1 ----------------
    kill_turtle1 = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/remove_turtle ",
            "turtlesim/srv/Kill ",
            '"name: turtle1"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(kill_turtle1)

    # ---------------- Spawn Killer ----------------
    spawn_killer = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: 1.0, y: 1.0, theta: 0.0, name: {killer_namespace}}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_killer)

    # ---------------- Spawn Eater ----------------
    spawn_eater = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: 3.0, y: 3.0, theta: 0.0, name: {eater_namespace}}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_eater)

    return launch_description