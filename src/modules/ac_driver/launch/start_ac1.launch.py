from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('ac_driver').find('ac_driver')
    angle_calib_basic_dir_path = os.path.join(pkg_share, 'conf/angle/')
    device_calib_file_path = os.path.join(pkg_share, 'conf/calibration.yaml')

    rviz_config_file = os.path.join(
        get_package_share_directory('ac_driver'),
        'rviz',
        'rviz2_config.rviz'
    )
        
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'), 
        Node(
            package='ac_driver',
            executable='ms_node',
            name='ms_node',
            output='screen', 
            parameters=[{
                'device_interface': "usb", # "usb"/"gmsl"
                'image_input_fps': 30,
                'imu_input_fps': 200,
                'enable_jpeg': True,
                'enable_rectify': True, 
                'enable_rectify_jpeg': True,
                'jpeg_quality': 70, 
                'topic_prefix': "",
                'serial_number': "",  
                'gmsl_device_number': "/dev/video30", 
                'point_frame_id': "rslidar", 
                'ac1_image_frame_id': "rslidar", 
                'ac2_left_image_frame_id': "rslidar", 
                'ac2_right_image_frame_id': "rslidar", 
                'imu_frame_id': "rslidar", 
                'enable_angle_and_device_calib_info_from_device': True, 
                'angle_calib_basic_dir_path': "",
                'enable_device_calib_info_from_device_pripority': True,
                'device_calib_file_path': "", 
                'device_manager_debug': False,
                'enable_use_lidar_clock': False, 
                'ros_stamp_compensate_ns': 0, # not support now !!!
                'enable_use_dense_points': False, 
                'enable_use_first_point_ts': False, 
                'enable_ac2_pointcloud_wave_split': False, 
                'enable_ros2_zero_copy': False, 
                'timestamp_output_dir_path': "", 
                'enable_pointcloud_send': True, 
                'enable_ac1_image_send': True, 
                'enable_ac2_left_image_send': False, 
                'enable_ac2_right_image_send': False, 
                'enable_imu_send': True, 
                # ac1 image crop 
                'ac1_crop_top': 0, 
                'ac1_crop_bottom': 0, 
                'ac1_crop_left': 0, 
                'ac1_crop_right': 0,
                # ac2 left image crop 
                'ac2_left_crop_top': 0, 
                'ac2_left_crop_bottom': 240, 
                'ac2_left_crop_left': 320, 
                'ac2_left_crop_right': 0,
                # ac2 right image crop 
                'ac2_right_crop_top': 0, 
                'ac2_right_crop_bottom': 240, 
                'ac2_right_crop_left': 320, 
                'ac2_right_crop_right': 0,
                # ac2_denoise_param 
                'enable_denoise': False,  
                'enable_smooth': False,  
                'dist_x_win_cfg': 1,  
                'dist_y_win_cfg': 1, 
                'dist_valid_thresholds_0': 3, 
                'dist_valid_thresholds_1': 3, 
                'dist_valid_thresholds_2': 3, 
                'dist_valid_thresholds_3': 2, 
                'dist_valid_thresholds_4': 2, 
                'max_process_distance': 65535, 
                'min_process_distance': 0, 
                # ac2_edge_param 
                'edge_kernel_size': 3, 
                # ac2_debloom_filter_param  
                'enable_debloom': False, 
                'search_range': 3,  
                'distance_diff_threshold': 50,
                'delete_intensity_threshold': 200,   
                'edge_thresholds_0': 2000,
                'edge_thresholds_1': 200,  
                'intensity_mutation_thresholds_0': 50,  
                'intensity_mutation_thresholds_1': 36,  
                'target_intensity_thresholds_0': 10, 
                'target_intensity_thresholds_1': 45, 
                'target_intensity_thresholds_2': 100,  
                # ac2_detrail_param 
                'enable_detrail': False, 
                'fwhm_thresholds_0': 95, 
                'fwhm_thresholds_1': 90, 
                'fwhm_thresholds_2': 85, 
                'fwhm_thresholds_3': 80,
                'edge_thresholds_0': 200, 
                'edge_thresholds_1': 500, 
                'edge_thresholds_2': 4000, 
                'distance_thresholds_0': 600, 
                'distance_thresholds_1': 1000,
                'distance_thresholds_2': 1500,
                'noise_thresholds_0': 15, 
                'noise_thresholds_1': 200, 
                'peak_value_threshold': 10, 
                'detrail_distance_diff_threshold': 30, 
                # ac2_frame_filter_param 
                'enable_frame_filter': False, 
                'enable_save_raw_data': False, 
                'smooth_frame_count': 5, 
                'imu_motion_detect_frame_count': 5, 
                'imu_motion_threshold': 3, 
                'stationary_ratio': 10,  
            }]
            #, prefix='gdb -ex run --args', 
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
