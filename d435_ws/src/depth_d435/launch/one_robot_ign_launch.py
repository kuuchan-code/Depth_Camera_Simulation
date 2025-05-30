import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# Add Command and FindExecutable imports
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

robot_model = 'sensors_diffbot'
robot_ns = 'r1' # Robot namespace (robot name)
pose = ['1.0', '0.0', '0.0', '0.0'] #Initial robot pose: x,y,z,th
robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)
world_file = 'warehouse.sdf' # warehouse

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory('depth_d435'))

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Set ign sim resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/ns_robot.rviz")],
    )

    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r'])

        ]
    )

    # --- CHANGES START HERE ---

    # Define the Xacro file path
    xacro_file_path = os.path.join(this_pkg_path, 'urdf', robot_model + '.xacro')

    # Create a LaunchConfiguration for the base_color and ns if they are used in Xacro with $(arg)
    # However, since they are static strings here, we can pass them directly as Command args.
    # If 'base_color' and 'ns' were launch arguments passed to this launch file,
    # you would use LaunchConfiguration('base_color') here.
    
    # Process the Xacro file using Command
    # This will execute 'xacro' as a command-line tool, resolving arguments within the Xacro file
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ', xacro_file_path,
            ' base_color:=', robot_base_color,
            ' ns:=', robot_ns # Pass namespace as an argument to xacro
        ]
    )

    # Now pass the processed robot_description_content to robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns, # This is the node namespace, not the URDF namespace
        output="screen",
        parameters=[{'robot_description': robot_description_content}] # Use the Command here
    )

    # For gz_spawn_entity, it also needs the resolved URDF.
    # We can use the same Command for it.
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content, # Use the Command here
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_ns,
                   '-allow_renaming', 'false'],
    )

    # --- CHANGES END HERE ---

    # Bridge (remains the same)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/model/'+robot_ns+'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/'+robot_ns+'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/world_model/model/'+robot_ns+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

            # '/'+robot_ns+'/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/'+robot_ns+'/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            
            '/'+robot_ns+'/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/'+robot_ns+'/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/'+robot_ns+'/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/'+robot_ns+'/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/'+robot_ns+'/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/'+robot_ns+'/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/model/'+robot_ns+'/battery/linear_battery/state@sensor_msgs/msg/BatteryState[ignition.msgs.BatteryState',
        ],
        parameters=[{'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[            # ign topic -l
            ('/model/'+robot_ns+'/cmd_vel', '/'+robot_ns+'/cmd_vel'),
            ('/model/'+robot_ns+'/odometry', '/'+robot_ns+'/odom'),
            ('/world/world_model/model/'+robot_ns+'/joint_state', '/'+robot_ns+'/joint_states'),

            # ('/'+robot_ns+'/imu', '/'+robot_ns+'/imu'),
            ('/'+robot_ns+'/lidar', '/'+robot_ns+'/scan'), 
            ('/model/'+robot_ns+'/battery/linear_battery/state', '/'+robot_ns+'/battery/state'),
        ]
    )


    return LaunchDescription(
        [
            simu_time,
            ign_resource_path,
            open_rviz,
            open_ign,
            gz_spawn_entity,
            robot_state_publisher,
            bridge
        ]
    )