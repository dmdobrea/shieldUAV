from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
			package='kamikaze_uav',
			executable='videoNode',
			name='video_pub_node',
			output='screen',
			emulate_tty=True,
			parameters=[
                {'verbose':   1},
                {'videoSource':  0},
                {'file_path': 'None'},        # path to the video, like '/home/user/FilmTest.mp4' or 'None'
                {'width':     640},
                {'height':    480}
            ]
        ),
        
        Node(
			package='kamikaze_uav',
			executable='broadcastNode',
			name='video_broadcast_node',
			output='screen',
			emulate_tty=True,
			parameters=[
                {'verbose':     1},
                {'server_ip':  '192.168.100.117'},
                {'source_sel':  1}                 # 0 - from videp_publisher_node, 1 - from fvideo_detection_node
            ]
        ),        
        
        Node(
			package='kamikaze_uav',
			executable='detectionNode',
			name='video_det_node',
			output='screen',
			emulate_tty=True,
			parameters=[
                {'verbose':     1}
            ]
        ),        
        
        Node(
			package='kamikaze_uav',
			executable='velocityNode',
			name='velocity_ctrl_node',
			output='screen',
			emulate_tty=True,
			parameters=[
			]
        ),
        
        Node(
			package='kamikaze_uav',
			executable='controlNode',
			name='main_control_node',
			output='screen',
			emulate_tty=True,
			parameters=[
                {'verbose':    1}
            ]
        ),  
    ])
