"""
巡逻机器人节点
功能：按预设路径点巡逻，到达每个点后判断所在区域，
      原地转向4个方向各拍一张照片，语音播报全程伴随。
"""

import rclpy
import os
import time
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
# 语音合成服务接口
from autopartol_interfaces.srv import SpeachText
# 真机相机相关依赖
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PatrolNode(BasicNavigator):
    """巡逻节点，继承BasicNavigator实现导航功能"""

    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)

        # ========== 1. 导航参数定义 ==========
        # 初始位姿 [x, y, yaw]
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        # 巡逻目标点，每3个数一组 [x, y, yaw]
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        # ========== 2. TF 监听器（获取实时位置）==========
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        # ========== 3. 语音合成客户端 ==========
        self.speach_client_ = self.create_client(SpeachText, 'speak_text')

        # ========== 4. 真机相机参数 ==========
        self.declare_parameter('camera_topic', '/fishbot_camera_raw')   # 相机图像话题
        self.declare_parameter('photo_save_dir', 'patrol_photos')       # 照片保存目录
        self.declare_parameter('enable_camera', True)                   # 是否启用相机
        self.camera_topic_ = self.get_parameter('camera_topic').value
        self.photo_save_dir_ = self.get_parameter('photo_save_dir').value
        self.enable_camera_ = self.get_parameter('enable_camera').value
        # OpenCV 图像转换桥
        self.cv_bridge = CvBridge()
        # 缓存最新一帧相机图像
        self.latest_image = None
        self.image_received = False
        # 拍照序号，用于文件名
        self.photo_index = 0

        # ========== 5. 区域定义（用于判断到了哪个房间）==========
        # 区域名称列表，和 area_bounds 一一对应
        self.declare_parameter('area_names', ['走廊', '厨房', '客厅', '卫生间'])
        # 每个区域4个数：x_min, x_max, y_min, y_max（矩形边界）
        self.declare_parameter('area_bounds', [
            -1.0, 1.0, -1.0, 1.0,
            4.0, 7.0, 1.5, 4.5,
            -8.0, -4.0, 2.0, 5.0,
            -11.0, -8.5, 0.0, 2.5,
        ])
        self.area_names_ = self.get_parameter('area_names').value
        self.area_bounds_ = self.get_parameter('area_bounds').value

        # ========== 6. 初始化相机订阅 ==========
        if self.enable_camera_:
            # 创建照片保存目录（不存在则自动创建）
            os.makedirs(self.photo_save_dir_, exist_ok=True)
            self.get_logger().info(f'照片保存目录: {os.path.abspath(self.photo_save_dir_)}')
            # 订阅相机图像话题，收到图像时调用 image_callback
            self.image_sub_ = self.create_subscription(
                Image,
                self.camera_topic_,
                self.image_callback,
                10
            )
            self.get_logger().info(f'已订阅相机话题: {self.camera_topic_}')

    def image_callback(self, msg):
        """相机图像回调：将 ROS Image 消息转为 OpenCV 格式并缓存"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {str(e)}')

    def record_image(self, point_name=None):
        """保存当前缓存的一帧图像到文件"""
        if not self.enable_camera_:
            self.get_logger().warn('相机功能未启用')
            return False
        if not self.image_received or self.latest_image is None:
            self.get_logger().warn('尚未收到相机图像，无法拍照')
            return False
        try:
            # 文件名格式：时间戳_序号_区域名.jpg
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            if point_name:
                filename = f'{timestamp}_{self.photo_index}_{point_name}.jpg'
            else:
                filename = f'{timestamp}_{self.photo_index}.jpg'
            filepath = os.path.join(self.photo_save_dir_, filename)
            # 用 OpenCV 保存图片
            cv2.imwrite(filepath, self.latest_image)
            self.photo_index += 1
            self.get_logger().info(f'照片已保存: {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'保存照片失败: {str(e)}')
            return False

    def get_area_name(self, x, y):
        """根据坐标(x,y)判断当前在哪个区域，返回区域名或None"""
        for i in range(len(self.area_names_)):
            base = i * 4
            if base + 3 >= len(self.area_bounds_):
                break
            x_min = self.area_bounds_[base]
            x_max = self.area_bounds_[base + 1]
            y_min = self.area_bounds_[base + 2]
            y_max = self.area_bounds_[base + 3]
            # 坐标落在矩形范围内，返回该区域名
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return self.area_names_[i]
        return None

    def speach_text(self, text):
        """调用语音合成服务播报文本"""
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语合成服务未上线，等待中。。。')
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f'语音合成成功：{text}')
            else:
                self.get_logger().warn(f'语音合成失败：{text}')
        else:
            self.get_logger().warn('语音合成服务请求失败')

    def get_pose_by_xyyaw(self, x, y, yaw):
        """通过 x, y, yaw 合成 PoseStamped（map坐标系下）"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 欧拉角转四元数
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def init_robot_pose(self):
        """初始化机器人位姿（设置初始位置并等待Nav2激活）"""
        self.initial_point_ = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        self.waitUntilNav2Active()

    def get_target_points(self):
        """从参数获取目标点集合，每3个数为一组 [x, y, yaw]"""
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index}->({x},{y},{yaw})')
        return points

    def nav_to_pose(self, target_pose):
        """导航到指定位姿，阻塞直到到达或失败"""
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        # 等待导航完成
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
        # 判断导航结果
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')

    def get_current_pose(self):
        """通过TF获取当前位姿（map -> base_footprint）"""
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')

    def rotate_and_capture(self, x, y, area_name):
        """
        原地转向4个方向拍照：
        0°(前方) -> 90°(右方) -> 180°(后方) -> 270°(左方)
        每个方向到达后停顿1秒再拍照，确保图像稳定
        """
        # 4个朝向角度（弧度）
        angles = [0.0, 1.57, 3.14, -1.57]
        # 4个方向的中文名称
        direction_names = ["前方", "右方", "后方", "左方"]
        for i, yaw in enumerate(angles):
            # 构造目标位姿（位置不变，只改朝向）
            pose = self.get_pose_by_xyyaw(x, y, yaw)
            # 导航转向（原地旋转）
            self.nav_to_pose(pose)
            # 停顿1秒等图像稳定
            time.sleep(1)
            # 拍照文件名带区域+方向，例如：客厅1_前方
            photo_name = f"{area_name}_{direction_names[i]}"
            self.speach_text(text=f"{direction_names[i]}拍照")
            success = self.record_image(point_name=photo_name)
            if success:
                self.get_logger().info(f'{direction_names[i]}拍照完成')
            else:
                self.get_logger().warn(f'{direction_names[i]}拍照失败')


def main():
    rclpy.init()
    patrol = PatrolNode()

    # 1. 初始化位姿
    patrol.speach_text(text='正在初始化位置')
    patrol.init_robot_pose()
    patrol.speach_text(text='位置初始化完成')

    # 2. 主循环：按路径点巡逻
    while rclpy.ok():
        for point in patrol.get_target_points():
            x, y, yaw = point[0], point[1], point[2]

            # 2.1 导航到目标点
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speach_text(text=f'准备前往目标点{x},{y}')
            patrol.nav_to_pose(target_pose)

            # 2.2 到达后判断区域名
            area_name = patrol.get_area_name(x, y)
            if area_name:
                patrol.speach_text(text=f"已到达{area_name}，准备环绕拍照")
                photo_name = area_name
            else:
                patrol.speach_text(text=f"已到达目标点{x},{y}，准备环绕拍照")
                photo_name = f'point_{x}_{y}'.replace('.', '_')

            # 2.3 原地转4个方向拍照
            patrol.rotate_and_capture(x, y, photo_name)

            # 2.4 播报该点拍照完成
            patrol.speach_text(text=f"{photo_name}环绕拍照完成")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
