from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    imu_data_node = Node(
        package="ml3_ros2",
        executable="imu_data_v1"
    )

    imu_sub_node = Node(
        package="ml3_ros2",
        executable="imu_sub"
    )

    ir_data_node = Node(
        package="ml3_ros2",
        executable="ir_read_data"
    )

    ir_sub_node = Node(
        package="ml3_ros2",
        executable="ir_data_sub"
    )

    qr_data_node = Node(
        package="ml3_ros2",
        executable="qr_data"
    )

    qr_sub_node = Node(
        package="ml3_ros2",
        executable="qr_sub"
    )

    ultra_data_node = Node(
        package="ml3_ros2",
        executable="ultra_data"
    )

    ultra_sub_node = Node(
        package="ml3_ros2",
        executable="ultra_sub"
    )

    ld.add_action(imu_data_node)
    ld.add_action(imu_sub_node)
    ld.add_action(ir_data_node)
    ld.add_action(ir_sub_node)
    ld.add_action(qr_data_node)
    ld.add_action(qr_sub_node)
    ld.add_action(ultra_data_node)
    ld.add_action(ultra_sub_node)

    return ld