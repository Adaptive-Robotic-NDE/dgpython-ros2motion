from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    dgpython_rosmotion_package = LaunchConfiguration("dgpython_rosmtion")

    scannertestcfgfile = PathJoinSubstitution(
        [FindPackageShare(dgpython_rosmotion_package), "demos", "scannertest.dgp"]
    )

    executable_description = ExecuteProcess(
        cmd = [['dataguzzler-python',
                scannertestcfgfile]]
    )

    return LaunchDescription([
        executable_description
    ])