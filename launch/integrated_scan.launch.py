#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction, GroupAction
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def launch_nodes_withconfig(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')

    #general parameter for the integrated laserscan
    pointCloudTopic = LaunchConfiguration('integratedTopic')
    pointCloutFrameId = LaunchConfiguration('integratedFrameId')

    #parameter for the first laserscan, feel free to duplicate and rename for other laserscans
    scanTopic1 = LaunchConfiguration('scanTopic1')
    laser1XOff = LaunchConfiguration('laser1XOff')
    laser1YOff = LaunchConfiguration('laser1YOff')
    laser1Alpha = LaunchConfiguration('laser1Alpha')
    show1 = LaunchConfiguration('show1')

    #parameter for the second laserscan, feel free to duplicate and rename for other laserscans
    scanTopic2 = LaunchConfiguration('scanTopic2')
    laser2XOff = LaunchConfiguration('laser2XOff')
    laser2YOff = LaunchConfiguration('laser2YOff')
    laser2Alpha = LaunchConfiguration('laser2Alpha')
    show2 = LaunchConfiguration('show2')

    robotFrontEnd = LaunchConfiguration('robotFrontEnd')
    robotRearEnd = LaunchConfiguration('robotRearEnd')
    robotRightEnd = LaunchConfiguration('robotRightEnd')
    robotLeftEnd = LaunchConfiguration('robotLeftEnd')

    #header of the output scan topic
    rangeMin = LaunchConfiguration('rangeMin')
    rangeMax = LaunchConfiguration('rangeMax')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    load_nodes = GroupAction(
        actions=[
            Node(package='laser_scan_integrator',
                    executable='laser_scan_integrator',
                    parameters=[{
                        'integratedTopic' : '/'+launch_configuration['namespace']+'/scan',
                        'integratedFrameId' : launch_configuration['namespace']+'/laser_link',
                        'scanTopic1' : '/'+launch_configuration['namespace']+launch_configuration["scanTopic1"],
                        'laser1XOff' : laser1XOff,
                        'laser1YOff' : laser1YOff,
                        'laser1Alpha' : laser1Alpha,
                        'show1' : show1,
                        'scanTopic2' : '/'+launch_configuration['namespace']+launch_configuration["scanTopic2"],
                        'laser2XOff' : laser2XOff,
                        'laser2YOff' : laser2YOff,
                        'laser2Alpha' : laser2Alpha,
                        'show2' : show2,
                        'robotFrontEnd' : robotFrontEnd,
                        'robotRearEnd' : robotRearEnd,
                        'robotRightEnd' : robotRightEnd,
                        'robotLeftEnd' : robotLeftEnd,
                        'rangeMin': rangeMin,
                        'rangeMax': rangeMax,
                    }],
                    namespace=namespace,
                    output='screen',
                    respawn=True,
                    respawn_delay=2,),
        ])
    
    return [load_nodes]

def generate_launch_description():

    declare_namespace_argument = DeclareLaunchArgument('namespace',  default_value='',
        description='Namespace of topics and services',
    )

    declare_integratedtopic_argument = DeclareLaunchArgument('integratedTopic', default_value="/robotinobase1/scan",
        description='Integrated topic to publish the laserscan to',
    )

    declare_integratedframe_argument = DeclareLaunchArgument('integratedFrameId', default_value="robotinobase1/laser_link",
        description='Integrated Frame ID',
    )

    declare_scantopic1_argument = DeclareLaunchArgument('scanTopic1', default_value="/front/sick_scan/scan",
        description='Scan topic of the first laserscan',
    )

    declare_laser1xoff_argument = DeclareLaunchArgument('laser1XOff', default_value='0.0',
        description='Offset of the first laserscan in x direction',
    )

    declare_laser1yoff_argument = DeclareLaunchArgument('laser1YOff', default_value='0.0',
        description='offset of the first laserscan in y direction',
    )

    declare_laser1aoff_argument = DeclareLaunchArgument('laser1Alpha', default_value='0.0',
        description='Rotation of the first laserscan in rad',
    )

    declare_showlaser1_argument = DeclareLaunchArgument('show1', default_value='True',
        description='Show the first laserscan in rviz',
    )

    declare_scantopic2_argument = DeclareLaunchArgument('scanTopic2', default_value="/back/sick_scan/scan",
        description='Scan topic of the second laserscan',
    )

    declare_laser2xoff_argument = DeclareLaunchArgument('laser2XOff', default_value='0.0',
        description='Offset of the second laserscan in x direction',
    )

    declare_laser2yoff_argument = DeclareLaunchArgument('laser2YOff', default_value='0.0',
        description='offset of the second laserscan in y direction',
    )

    declare_laser2aoff_argument = DeclareLaunchArgument('laser2Alpha', default_value='0.0',
        description='Rotation of the second laserscan in rad',
    )

    declare_showlaser2_argument = DeclareLaunchArgument('show2', default_value='True',
        description='Show the second laserscan in rviz',
    )

    declare_frontend_argument = DeclareLaunchArgument('robotFrontEnd', default_value='0.1',
        description='robotFrontEnd',
    )

    declare_rearend_argument = DeclareLaunchArgument('robotRearEnd', default_value='0.1',
        description='robotRearEnd',
    )

    declare_rightend_argument = DeclareLaunchArgument('robotRightEnd', default_value='0.1',
        description='robotRightEnd',
    )

    declare_leftend_argument = DeclareLaunchArgument('robotLeftEnd', default_value='0.1',
        description='robotLeftEnd',
    )

    declare_rangeMin_argument = DeclareLaunchArgument('rangeMin', default_value='0.225',
        description='rangeMin',
    )

    declare_rangeMax_argument = DeclareLaunchArgument('rangeMax', default_value='100.0',
        description='rangeMax',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_integratedtopic_argument)
    ld.add_action(declare_integratedframe_argument)
    ld.add_action(declare_scantopic1_argument)
    ld.add_action(declare_laser1xoff_argument)
    ld.add_action(declare_laser1yoff_argument)
    ld.add_action(declare_laser1aoff_argument)
    ld.add_action(declare_showlaser1_argument)
    ld.add_action(declare_scantopic2_argument)
    ld.add_action(declare_laser2xoff_argument)
    ld.add_action(declare_laser2yoff_argument)
    ld.add_action(declare_laser2aoff_argument)
    ld.add_action(declare_showlaser2_argument)
    ld.add_action(declare_frontend_argument)
    ld.add_action(declare_rearend_argument)
    ld.add_action(declare_rightend_argument)
    ld.add_action(declare_leftend_argument)
    ld.add_action(declare_rangeMax_argument)
    ld.add_action(declare_rangeMin_argument)
    # Add the actions to launch all of the localiztion nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
