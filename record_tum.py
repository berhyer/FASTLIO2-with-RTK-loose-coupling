import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import sys

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        # 你可以根据需要修改这里的话题名和输出文件名
        # 如果你想存纯 LIO 的轨迹，用 '/lio_path'
        # 如果你想存 PGO 优化后的轨迹，用你 PGO 节点发出的 path 话题
        self.declare_parameter('path_topic', '/lio_path') 
        self.declare_parameter('output_file', 'pgo_traj.txt')
        
        self.topic_name = self.get_parameter('path_topic').value
        self.file_name = self.get_parameter('output_file').value
        
        self.subscription = self.create_subscription(
            Path,
            self.topic_name,
            self.path_callback,
            10)
        self.get_logger().info(f"等待接收话题 {self.topic_name} 的轨迹数据...")
        self.get_logger().info(f"提示: 在播放完数据集后，按下 [Ctrl+C] 即可自动将轨迹保存为 TUM 格式！")
        self.latest_path = None

    def path_callback(self, msg):
        self.latest_path = msg
        # 为了防止刷屏，这里不打印每一帧，你可以根据需要取消注释
        # self.get_logger().info(f"已接收 {len(msg.poses)} 个位姿节点")

    def save_to_file(self):
        if self.latest_path is None:
            self.get_logger().warn("没有接收到任何轨迹数据，未保存文件。")
            return
            
        with open(self.file_name, 'w') as f:
            for pose_stamped in self.latest_path.poses:
                # 提取时间戳
                sec = pose_stamped.header.stamp.sec
                nanosec = pose_stamped.header.stamp.nanosec
                timestamp = sec + nanosec * 1e-9
                
                # 提取位置
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                z = pose_stamped.pose.position.z
                
                # 提取姿态 (四元数)
                qx = pose_stamped.pose.orientation.x
                qy = pose_stamped.pose.orientation.y
                qz = pose_stamped.pose.orientation.z
                qw = pose_stamped.pose.orientation.w
                
                # TUM 格式标准: timestamp x y z qx qy qz qw
                f.write(f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
                
        self.get_logger().info(f"✅ 成功! 已将 {len(self.latest_path.poses)} 个位姿保存至: {self.file_name}")

def main(args=None):
    rclpy.init(args=args)
    recorder = PathRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("检测到 Ctrl+C，正在保存轨迹...")
    finally:
        recorder.save_to_file()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()