from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	
	node = Node(
		package="bevfusion",  
		executable="bevfusion_node",  
		name='bevfusion_node',       
		output='screen', 
		
		# 参数列表
		parameters=[
			{'model_name': 'resnet50int8'},
			{'precision' : 'int8'}
		]
	)

	# launch中启动没有读到rviz配置文件 可以自己修改
	# rviz_node = Node(
    #         package='rviz2',
    #         namespace='',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments='-d src/BEVFusion-ROS-TensorRT/launch/view.rviz'
	# )

	return LaunchDescription([
		node,
		# rviz_node
	])