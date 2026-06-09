import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

class RTKPlayer(Node):
    def __init__(self, bag_path):
        super().__init__('rtk_player')
        
        # 开启仿真时间，强制跟随 ros2 bag play 的进度条
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(NavSatFix, '/rtk_fix', 10)
        self.gnss_msgs = []
        
        self.get_logger().info('🚀 正在潜入原始 Bag 文件，提取作者的私有数据结构...')
        
        typestore = get_typestore(Stores.ROS1_NOETIC)
        
        with Reader(bag_path) as reader:
            # ================================================================
            # 🌟 修复新版 rosbags 库的字符串对象问题，提取纯文本
            # ================================================================
            type_dict = {}
            for connection in reader.connections:
                if connection.topic == '/novatel_data/inspvax':
                    if connection.msgtype not in typestore.fielddefs:
                        # 兼容新老版本：如果是对象就取 .data，如果是字符串就直接用
                        raw_msgdef = connection.msgdef.data if hasattr(connection.msgdef, 'data') else connection.msgdef
                        type_dict.update(get_types_from_msg(raw_msgdef, connection.msgtype))
            
            if type_dict:
                typestore.register(type_dict)
                self.get_logger().info('✅ 成功将原作者的数据结构烙印进内存！不再发生错位！')

            # ================================================================
            # 提取数据
            # ================================================================
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/novatel_data/inspvax':
                    # 现在，真正的内存级对齐解锁了！
                    msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                    
                    fix = NavSatFix()
                    fix.header.stamp.sec = timestamp // 10**9
                    fix.header.stamp.nanosec = timestamp % 10**9
                    fix.header.frame_id = 'gnss'
                    
                    # 提取真实的经纬高
                    fix.latitude = msg.latitude
                    fix.longitude = msg.longitude
                    fix.altitude = msg.altitude
                    
                    fix.position_covariance[0] = msg.latitude_std ** 2
                    fix.position_covariance[4] = msg.longitude_std ** 2
                    fix.position_covariance[8] = msg.altitude_std ** 2
                    fix.status.status = 0
                    
                    self.gnss_msgs.append((timestamp, fix))
                    
        self.get_logger().info(f'✅ 成功提取并武装了 {len(self.gnss_msgs)} 条 RTK 精确弹药！')
        
        self.current_idx = 0
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if self.current_idx >= len(self.gnss_msgs):
            return
            
        now = self.get_clock().now().nanoseconds
        if now <= 0:
            return
            
        while self.current_idx < len(self.gnss_msgs) and now >= self.gnss_msgs[self.current_idx][0]:
            self.pub.publish(self.gnss_msgs[self.current_idx][1])
            self.current_idx += 1

def main():
    rclpy.init()
    # 🚨 注意：这里填原始 ROS 1 .bag 文件的绝对或相对路径
    bag_path = '/mnt/hgfs/Ubuntu_sharefile/fastlioRTK_test/UrbanNav-HK_TST-20210517_sensors.bag' 
    node = RTKPlayer(bag_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
