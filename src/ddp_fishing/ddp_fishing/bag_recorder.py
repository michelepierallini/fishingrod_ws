import os
import re

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import rosbag2_py
from rosbag2_py import Recorder, RecordOptions
from ddp_fishing.utils import get_csv_filepath


class BagRecorder(Node):
    """Utility node to inspect the content of the /joint_states messages in a bag file."""
    
    def __init__(self):
        super().__init__('BagRecorder')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bag_filename', '010_move_base'),
                ('parent_folder', 'periodic'),
                ('rate', 1.),
            ]
        )
        
        self.declare_parameter('time', 'yyyy-mm-dd-hh-mm-ss')
        time = str(self.get_parameter('time').get_parameter_value().string_value)
        
        bag_filename_ = str(self.get_parameter('bag_filename').get_parameter_value().string_value)
        # Erase file extension
        bag_filename = re.sub(r'(\.[^.]+)$', '', bag_filename_)
        parent_folder = str(self.get_parameter('parent_folder').get_parameter_value().string_value)
        directory = f"{get_package_share_directory('ddp_controller_publisher')}/../../../../../tasks/"
        workspace_directory = os.path.join(directory, parent_folder)
        bag_filepath = f"{workspace_directory}/rosbags/bag_{time}-{bag_filename}"
        
                
        # Create the bag reader.
        self.recorder = Recorder()
        
        self.record_options = RecordOptions()
        self.record_options.all = True
        # self.record_options.topics = ['/joint_states', '/PD_control/command']
        
        self.storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_filepath, storage_id='mcap')
        
        self.get_name()
        
    def __call__(self):
        try:
            self.recorder.record(self.storage_options, self.record_options)
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    recorder = BagRecorder()
    recorder()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
