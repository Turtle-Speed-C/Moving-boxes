import threading
import signal
import rospy
import sys
import time
import os
from std_msgs.msg import String, Float64MultiArray
from amr.WooshWebSocketClient import WooshApi
from ros_functions.control_arm_demo import arm_pose_publish
from ros_functions.tag_info_stabe_v3 import get_apriltag_average_data
from ros_functions.control_head_demo import head_motion
from amr.LiftControlClient import LiftControlClient
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from geometry_msgs.msg import Pose2D
from ruamel.yaml import YAML
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup import init_logging

from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy_v2 import (
    execute_joint_trajectory,
    arm_for_detector_code,
)

# 基础路径配置
mother_dir = os.path.dirname(os.path.abspath(__file__))
log_path = init_logging(log_dir=os.path.join(mother_dir, "logs"), filename_prefix="mes_box", enable=True)

class BoxPickPlaceDemo:
    """
    机械臂抓放箱任务主类
    核心流程：初始化→AprilTag识别→条码扫描→MES等待→抓取→搬运→放置→任务完成
    """
    def __init__(self, ws_addr):
        # ROS节点初始化
        rospy.init_node('catch_demo') 
        self.amr = WooshApi(ws_addr)
        self.lift_client = LiftControlClient()
        self.robot_sdk = RobotSDK()

        # 功能模块初始化
        self.arm_publisher = arm_pose_publish
        self.get_aprtag = get_apriltag_average_data
        self.head_publisher = head_motion
        
        # 位置/库位状态订阅
        self.sub = rospy.Subscriber('/position1', Pose2D, self._position1_callback)
        self.sub_loc = rospy.Subscriber('/position_location', String, self._location_callback)
        self.position1_received = False
        self.location_received = False
        self.position1_data = None
        self.location_code = None

        # 条形码订阅
        self.barcode_sub = rospy.Subscriber('/barcode_detection_result', String, self._barcode_callback)
        self.latest_barcode_data = None
        self.barcode_received = False

        # 电机电流订阅（掉箱/重量检测）
        self.motor_current_sub = rospy.Subscriber('/sensor_data_motor/motor_cur', Float64MultiArray, self._motor_current_callback)
        self.latest_motor_current = None
        self.motor_current_received = False

        # 加载配置文件
        config = self._load_yaml('/home/lab/12_1_Goertek/kuavo-ros-control/src/wrc_demo/config/robot_config_new.yaml')
        self.arm_actions = config.get('arm_actions', {})
        self.robot_params = config.get('robot_params', {}) or {}

        # 解析所有配置
        self._parse_configurations()
        
        # ✨ 添加缺失的变量初始化
        self.task_state = 'running'
        self.box_carrying = False
        self.is_real_drop = False
        self.detected_weight_level = None
        
        # 机械臂动作时间配置
        self.arm_action_times = [1]  # 默认3秒，可以改成配置文件读取
        
        # 升降台速度倒数（用于计算等待时间）
        self.height_speed_reciprocal = 3.0  # 1/0.25 = 4秒每米
        
        # 掉箱检测相关
        self.current_history = []
        self.max_history_length = 1000
        self._last_print_time = 0
        self.drop_detection_log_file = os.path.join(mother_dir, "logs", "drop_detection.csv")
        
        # AprilTag偏移量存储（阶段2获取，阶段5使用）
        self.apriltag_offsets = {
            'horizontal_offset': 0.0,
            'vertical_offset': 0.0,
            'off_camera': 0.0
        }
        
        # 初始化配置打印
        print("-" * 60)
        print("初始化配置信息")
        print("-" * 60)
        print(f"掉箱检测配置：")
        print(f"  - 电流阈值12: {self.drop_current_threshold_12}")
        print(f"  - 电流阈值13: {self.drop_current_threshold_13}")
        print(f"  - 检测间隔: {self.drop_check_interval}秒")
        print(f"  - 连续判定次数: {self.drop_consecutive_counts}")
        print(f"重量检测配置：")
        print(f"  - 启用状态: {self.enable_weight_adjustment}")
        print(f"  - 轻箱子阈值: < {self.weight_light_threshold}")
        print(f"  - 重箱子阈值: > {self.weight_heavy_threshold}")
        print(f"  - 高度调整: 轻{self.weight_height_adjustments['light']*100:.1f}cm, 中{self.weight_height_adjustments['medium']*100:.1f}cm, 重{self.weight_height_adjustments['heavy']*100:.1f}cm")
        print("-" * 60)

    def reload_config(self):
        """重新加载配置文件"""
        print("重新加载配置文件...")
        try:
            config = self._load_yaml('/home/lab/12_1_Goertek/kuavo-ros-control/src/wrc_demo/config/robot_config_new.yaml')
            self.arm_actions = config.get('arm_actions', {})
            self.robot_params = config.get('robot_params', {}) or {}
            
            # 重新解析所有配置
            self._parse_configurations()
            
            # ✨ 添加：重载后也打印配置信息
            print("-" * 60)
            print("配置文件重新加载完成 - 新配置如下：")
            print("-" * 60)
            print(f"掉箱检测配置：")
            print(f"  - 电流阈值12: {self.drop_current_threshold_12}")
            print(f"  - 电流阈值13: {self.drop_current_threshold_13}")
            print(f"  - 检测间隔: {self.drop_check_interval}秒")
            print(f"  - 连续判定次数: {self.drop_consecutive_counts}")
            print(f"重量检测配置：")
            print(f"  - 启用状态: {self.enable_weight_adjustment}")
            print(f"  - 轻箱子阈值: < {self.weight_light_threshold}")
            print(f"  - 重箱子阈值: > {self.weight_heavy_threshold}")
            print(f"  - 高度调整: 轻{self.weight_height_adjustments['light']*100:.1f}cm, 中{self.weight_height_adjustments['medium']*100:.1f}cm, 重{self.weight_height_adjustments['heavy']*100:.1f}cm")
            print("-" * 60)
            
            return True
        except Exception as e:
            print(f"重新加载配置文件失败: {e}")
            return False
    
    def _load_yaml(self, yaml_file):
        """加载yaml配置文件"""
        with open(yaml_file, 'r') as file:
            return YAML().load(file)
    
    def _parse_configurations(self):
        """解析所有配置文件参数"""
        # 解析位置配置
        positions_cfg = self.robot_params.get('positions', {}) or {}
        self.position_catch = positions_cfg.get('catch_point', {'x': 0, 'y': 0, 'theta': 0})
        self.position_left_place = positions_cfg.get('place_left', {'x': 0, 'y': 0, 'theta': 0})
        self.position_right_place = positions_cfg.get('place_right', {'x': 0, 'y': 0, 'theta': 0})

        # 解析条码扫描配置
        barcode_cfg = self.robot_params.get('barcode_scan', {}) or {}
        self.barcode_scan_steps = {
            'ahead': barcode_cfg.get('ahead', {'distance': 0, 'speed': 0}),
            'left': barcode_cfg.get('left', {'distance': 0, 'speed': 0}),
            'back': barcode_cfg.get('back', {'distance': 0, 'speed': 0}),
            'right': barcode_cfg.get('right', {'distance': 0, 'speed': 0}),
        }
        self.barcode_scan_height = barcode_cfg.get('height', 0)

        # 解析AprilTag扫描配置
        apriltag_cfg = self.robot_params.get('apriltag_scan', {}) or {}
        self.apriltag_scan_steps = {
            'ahead': apriltag_cfg.get('ahead', {'distance': 0, 'speed': 0}),
            'left': apriltag_cfg.get('left', {'distance': 0, 'speed': 0}),
            'back': apriltag_cfg.get('back', {'distance': 0, 'speed': 0}),
            'right': apriltag_cfg.get('right', {'distance': 0, 'speed': 0}),
        }

        # 解析抓取配置
        catching_cfg = self.robot_params.get('catching', {}) or {}
        self.catch_height = catching_cfg.get('height', 0)
        self.catch_x_distance = catching_cfg.get('x_distance', 0)
        self.normal_travel_height = catching_cfg.get('travel_height', 0)

        # 解析放置配置
        placements_cfg = self.robot_params.get('placements', {}) or {}
        self.placement_layers = placements_cfg

        # 基础任务参数
        self.task_repeat = self.robot_params.get('task_repeat', 0)
        self.catched_leave_height = self.robot_params.get('catched_leave_height', 0)
        self.step_speed = self.robot_params.get('step_speed', 0)
        self.step_back_distance = self.robot_params.get('step_back_distance', 0)

        # 掉箱检测参数
        drop_cfg = self.robot_params.get('drop_detection', {}) or {}
        self.drop_current_threshold_12 = drop_cfg.get('threshold_12', 0.2)
        self.drop_current_threshold_13 = drop_cfg.get('threshold_13', 3.3)
        self.drop_check_interval = drop_cfg.get('check_interval', 0.1)
        self.drop_consecutive_counts = drop_cfg.get('consecutive_counts', 10)
        self.debug_drop_detection = drop_cfg.get('debug_mode', True)

        # 重量检测参数
        weight_cfg = self.robot_params.get('weight_detection', {}) or {}
        self.weight_height_adjustments = {
            'light': weight_cfg.get('light_height_offset', -0.02),
            'medium': weight_cfg.get('medium_height_offset', 0.0),
            'heavy': weight_cfg.get('heavy_height_offset', 0.0)
        }
        self.enable_weight_adjustment = weight_cfg.get('enable', True)
        self.weight_light_threshold = weight_cfg.get('light_threshold', 9.9)
        self.weight_heavy_threshold = weight_cfg.get('heavy_threshold', 10.3)
        
    def _cb_signal_handler(self, sig, frame):
        """处理 Ctrl+C 退出信号"""
        print('程序退出中...')
        self.task_state = 'stopped'
        self.box_carrying = False
        self.amr.close()
        sys.exit(0)

    def _cb_send_as_ping(self):
        """心跳包发送线程"""
        while self.task_state == 'running':
            print(self.amr.robot_battery())
            time.sleep(5)
        self.amr.close()

    def _cb_task_proc_callback(self, message):
        """消息回调处理"""
        print(f"收到消息: {message}")

    def _cb_add_subscriptions(self):
        """添加AMR话题订阅"""
        self.amr.add_topic_callback("woosh.robot.TaskProc", self._cb_task_proc_callback)
        self.amr.submit_subscriptions()
    
    def _cb_checkout_arrived(self):
        """等待机器人到达目标位置"""
        while True:
            try:
                if self.amr.robot_state()['state'] == 2:
                    print("  到达目标点")
                    return
            except Exception as e:
                print(f"  获取机器人状态失败：{e}")
            time.sleep(1)

    def _position1_callback(self, msg):
        """position1话题回调"""
        self.position1_received = True
        self.position1_data = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta
        }
        print(f"📍 [MES回调] 收到位置坐标: x={msg.x:.3f}, y={msg.y:.3f}, θ={msg.theta:.1f}")
        rospy.loginfo(f"收到position1消息: {self.position1_data}")

    def _location_callback(self, msg):
        """库位号回调"""
        self.location_received = True
        self.location_code = msg.data
        print(f"📍 [MES回调] 收到库位号: {self.location_code}")
        rospy.loginfo(f"收到库位号: {self.location_code}")

    def _barcode_callback(self, msg):
        """条形码检测回调"""
        self.latest_barcode_data = msg.data
        if msg.data:
            self.barcode_received = True
            print(f"  收到条形码: {msg.data}")
        else:
            print("  收到空条形码数据")

    def _motor_current_callback(self, msg):
        """电机电流回调（掉箱/重量检测）"""
        if len(msg.data) >= 14:
            self.latest_motor_current = msg.data
            self.motor_current_received = True
            
            current_12 = msg.data[12]
            current_13 = msg.data[13]
            timestamp = time.time()
            
            # 调试模式下记录电流数据
            if self.debug_drop_detection and self.box_carrying:
                log_entry = {
                    'time': timestamp,
                    'current_12': current_12,
                    'current_13': current_13,
                    'abs_12': abs(current_12),
                    'abs_13': abs(current_13),
                    'below_threshold_12': abs(current_12) < self.drop_current_threshold_12,
                    'below_threshold_13': abs(current_13) < self.drop_current_threshold_13
                }
                self.current_history.append(log_entry)
                
                if len(self.current_history) > self.max_history_length:
                    self.current_history.pop(0)
                
                # 控制打印频率
                if timestamp - self._last_print_time > 0.5:
                    print(f"[电流监控] 12: {current_12:.3f}, 13: {current_13:.3f} (阈值: {self.drop_current_threshold_13})")
                    self._last_print_time = timestamp
        else:
            self.motor_current_received = False
            self.latest_motor_current = None

    def reset_position_location_barcode_value_state(self):
        """重置位置/库位/条码状态"""
        self.position1_received = False
        self.location_received = False
        self.position1_data = None
        self.location_code = None
        self.reset_motor_current_state()
        if rospy.has_param('/barcode_value'):
            rospy.delete_param('/barcode_value')

    def reset_barcode_state(self):
        """重置条码检测状态"""
        self.barcode_received = False
        self.latest_barcode_data = None
        self.reset_motor_current_state()

    def reset_motor_current_state(self):
        """重置电流检测状态"""
        self.motor_current_received = False
        self.latest_motor_current = None

    def _wait_for_position1(self):
        """等待MES返回位置和库位信息"""
        print("  等待MES返回位置和库位信息...")
        wait_count = 0
        while (not self.position1_received or not self.location_received) and self.task_state == 'running':
            wait_count += 1
            if wait_count % 25 == 0:  # 每5秒打印一次等待状态
                position_status = "✓" if self.position1_received else "✗"
                location_status = "✓" if self.location_received else "✗"
                print(f"    等待中... 位置:{position_status}, 库位:{location_status} (已等待{wait_count*0.2:.1f}秒)")
            time.sleep(0.2)

        if self.position1_received and self.location_received:
            print(f"  ✅ 已收到MES消息 - 库位:{self.location_code}, 位置:x={self.position1_data['x']:.3f}, y={self.position1_data['y']:.3f}")
            return {
                "position": self.position1_data,
                "location": self.location_code
            }
        else:
            print("  ❌ 任务已停止，未收到完整的MES消息")
            return None

    def robot_move(self, pos):
        """机器人移动到指定位置"""
        self.amr.robot_go_to(x=pos['x'], y=pos['y'], theta=pos['theta'])
        time.sleep(1)
        self._cb_checkout_arrived()

    def arm_back(self):
        """机械臂复位到初始姿态"""
        left_arm_traj = [0, 0, 0, -90, 0, 0, 0]
        right_arm_traj = [0, 0, 0, -90, 0, 0, 0]
        self.arm_publisher(left_arm_traj, right_arm_traj, times=self.arm_action_times)

    def catch_box(self, height_type):
        """执行抓取动作"""
        actions = self.arm_actions[height_type]

        # 执行第一个抓取动作
        arm_action = actions[f'arm_action_1']
        self.arm_publisher(arm_action['left_arm_traj'], arm_action['right_arm_traj'], times=self.arm_action_times)
        time.sleep(self.arm_action_times[0] + 4)

        # height_1抓取时，第一个动作后降低升降台高度
        if height_type == "height_1":
            catching_cfg = self.robot_params.get('catching', {})
            catch_height_last = catching_cfg.get('catch_height_last', 0.24)
            self.lift_client.set_height(height=catch_height_last, speed=0.25)
            time.sleep(1.0)

        # 执行第二个抓取动作
        arm_action = actions[f'arm_action_2']
        self.arm_publisher(arm_action['left_arm_traj'], arm_action['right_arm_traj'], times=self.arm_action_times)
        time.sleep(0.5)

    def release_box(self, height_type):
        """执行释放动作"""
        actions = self.arm_actions[height_type]
        arm_action = actions[f'arm_action_1']
        self.arm_publisher(arm_action['left_arm_traj'], arm_action['right_arm_traj'], times=self.arm_action_times)
        time.sleep(self.arm_action_times[0] + 4)
        arm_action = actions[f'arm_action_2']
        self.arm_publisher(arm_action['left_arm_traj'], arm_action['right_arm_traj'], times=self.arm_action_times)
        time.sleep(0.5)
     
    def prepare_catch_box(self):
        """机械臂抓取前准备姿态"""
        left_arm_traj = [-10, 15, 10, -80, 0, 0, 0]
        right_arm_traj = [-10, -15, -10, -80, 0, 0, 0]
        self.arm_publisher(left_arm_traj, right_arm_traj, times=self.arm_action_times)
        time.sleep(3)

    def back_step(self):
        """机器人后退指定距离"""
        self.amr.robot_step_control(direction="back", distance=self.step_back_distance, speed=self.step_speed)
        time.sleep(self.step_back_distance*6/self.step_speed)

    def _is_box_possibly_dropped(self):
        """判断箱子是否可能掉落"""
        if not self.motor_current_received or self.latest_motor_current is None:
            return False
        if len(self.latest_motor_current) < 14:
            return False
        current_13 = self.latest_motor_current[13]
        abs_current_13 = abs(current_13)
        below_13 = abs_current_13 < self.drop_current_threshold_13
        return below_13

    def _drop_monitor_loop(self):
        """掉箱检测后台线程"""
        consecutive_low = 0
        loop_count = 0
        
        print("-" * 60)
        print("掉箱检测线程启动")
        print("-" * 60)
        
        while self.task_state == 'running' and self.box_carrying and not rospy.is_shutdown():
            loop_count += 1
            
            if not self.motor_current_received or self.latest_motor_current is None:
                if self.debug_drop_detection and loop_count % 20 == 0:
                    print(f"掉箱检测第{loop_count}次: 未收到电流数据")
                time.sleep(self.drop_check_interval)
                continue
            
            current_12 = self.latest_motor_current[12]
            current_13 = self.latest_motor_current[13]
            abs_12 = abs(current_12)
            abs_13 = abs(current_13)
            
            is_dropped = self._is_box_possibly_dropped()
            
            if self.debug_drop_detection and loop_count % 10 == 0:
                status = "可能掉落" if is_dropped else "正常"
                print(f"掉箱检测第{loop_count}次: 13={current_13:.3f} (abs={abs_13:.3f}) → {status}, 连续{consecutive_low}/{self.drop_consecutive_counts}")
            
            if is_dropped:
                consecutive_low += 1
                print(f"掉箱检测警告：疑似掉落！连续{consecutive_low}/{self.drop_consecutive_counts}")
                
                if consecutive_low >= self.drop_consecutive_counts:
                    if not self.box_carrying:
                        print("掉箱检测：box_carrying已为False，正常停止检测")
                        break
                    
                    print("-" * 60)
                    print("确认箱子已掉落！触发紧急停止！")
                    print(f"  电流12: {current_12:.3f}, 电流13: {current_13:.3f}")
                    print("-" * 60)
                    
                    self._save_current_history()
                    self.is_real_drop = True
                    
                    # 停止机器人运动
                    try:
                        self.amr.robot_step_control(direction='ahead', distance=0, speed=0, action=0)
                        print("掉箱检测：已停止机器人运动")
                    except Exception as e:
                        print(f"掉箱检测：停止机器人失败: {e}")
                    
                    # 停止升降台
                    try:
                        self.lift_client.publish_lift_goal(mode=0, execmode=1, speed=0.2, height=0)
                        print("掉箱检测：已停止升降台")
                    except Exception as e:
                        print(f"掉箱检测：停止升降台失败: {e}")
                    
                    # 更新任务状态
                    self.task_state = 'stopped'
                    self.box_carrying = False
                    
                    # 启动语音播报
                    if self.is_real_drop:
                        try:
                            barcode_info = self.latest_barcode_data if self.latest_barcode_data else "未知"
                            location_info = self.location_code if self.location_code else "未知"
                            self.speak_thread = threading.Thread(
                                target=self._speak_drop_alert,
                                args=(barcode_info, location_info),
                                daemon=False
                            )
                            self.speak_thread.start()
                            print("掉箱检测：已启动语音播报")
                            time.sleep(0.2)
                        except Exception as e:
                            print(f"掉箱检测：播报失败: {e}")
                    
                    # 设置任务状态参数
                    try:
                        rospy.set_param("/task_status", "box_dropped")
                    except Exception as e:
                        print(f"掉箱检测：设置参数失败: {e}")
                    
                    rospy.signal_shutdown("box dropped detected by current")
                    break
            else:
                if consecutive_low > 0:
                    print(f"掉箱检测：电流恢复，重置计数器（之前{consecutive_low}次）")
                consecutive_low = 0
            
            time.sleep(self.drop_check_interval)
        
        print(f"掉箱检测线程退出 (共检查{loop_count}次)")

    def _save_current_history(self):
        """保存电流历史数据到日志文件"""
        try:
            with open(self.drop_detection_log_file, 'w') as f:
                f.write("时间戳,电流12,电流13,绝对值12,绝对值13\n")
                for entry in self.current_history:
                    f.write(f"{entry['time']:.3f},{entry['current_12']:.3f},{entry['current_13']:.3f},"
                           f"{entry['abs_12']:.3f},{entry['abs_13']:.3f}\n")
            print(f"掉箱检测：电流历史已保存至 {self.drop_detection_log_file}")
        except Exception as e:
            print(f"掉箱检测：保存日志失败: {e}")
    
    def _detect_box_weight(self):
        """根据电机电流检测箱子重量等级"""
        if not self.enable_weight_adjustment:
            return None
        
        sample_count = 10
        sample_interval = 0.1
        current_13_samples = []
        
        print(f"  采样{sample_count}次检测箱子重量...")
        
        for i in range(sample_count):
            if not self.motor_current_received or self.latest_motor_current is None:
                time.sleep(sample_interval)
                continue
            
            if len(self.latest_motor_current) < 14:
                time.sleep(sample_interval)
                continue
            
            current_13 = self.latest_motor_current[13]
            abs_current_13 = abs(current_13)
            current_13_samples.append(abs_current_13)
            
            if i < sample_count - 1:
                time.sleep(sample_interval)
        
        if len(current_13_samples) == 0:
            print("  无法获取电流采样数据")
            return None
        
        # 去除极值后计算平均值
        if len(current_13_samples) >= 3:
            sorted_samples = sorted(current_13_samples)
            trimmed_samples = sorted_samples[1:-1]
        else:
            trimmed_samples = current_13_samples

        avg_current_13 = sum(trimmed_samples) / len(trimmed_samples)
        
        # 判断重量等级
        if avg_current_13 < self.weight_light_threshold:
            weight_level = 'light'
            weight_level_cn = '轻'
        elif avg_current_13 > self.weight_heavy_threshold:
            weight_level = 'heavy'
            weight_level_cn = '重'
        else:
            weight_level = 'medium'
            weight_level_cn = '中'
        
        print(f"  重量检测结果: {weight_level_cn} (平均电流{avg_current_13:.3f})")
        
        return weight_level
    
    def _adjust_height_by_weight(self, base_height, weight_level):
        """根据重量等级调整放置高度"""
        if not self.enable_weight_adjustment or weight_level is None:
            return base_height
        
        height_offset = self.weight_height_adjustments.get(weight_level, 0.0)
        adjusted_height = base_height + height_offset
        
        weight_level_cn_map = {'light': '轻', 'medium': '中', 'heavy': '重'}
        weight_level_cn = weight_level_cn_map.get(weight_level, weight_level)
        
        if abs(height_offset) > 0.001:
            print(f"  重量'{weight_level_cn}'，高度调整: {base_height:.3f}m → {adjusted_height:.3f}m (偏移{height_offset*100:.1f}cm)")
        else:
            print(f"  重量'{weight_level_cn}'，高度保持: {base_height:.3f}m")
        
        return adjusted_height
    
    def _speak_drop_alert(self, barcode_info, location_info):
        """掉箱警报语音播报"""
        print(f"掉箱检测：开始语音播报（5次）")
        
        parts = [
            "箱子，已掉落",
            f"库位，是{location_info}",
            "任务，已停止"
        ]
        
        for round_num in range(5):
            print(f"掉箱检测：第{round_num+1}轮播报")
            
            for part_idx, part in enumerate(parts):
                try:
                    self.amr.robot_speak(part)
                    print(f"  {part}")
                    time.sleep(2.5)
                    if part_idx < len(parts) - 1:
                        time.sleep(1.5)
                except Exception as e:
                    print(f"  播报失败: {e}")
            
            if round_num < 4:
                print("  等待10秒...")
                time.sleep(10)
        
        print("掉箱检测：播报完成")

    # ------------------------------
    # 任务流程阶段
    # ------------------------------

    def stage_1_initialization(self):
        """
        阶段1: 初始化
        - 升降台复位到安全高度
        - 重置状态标志
        - 导航到抓取点
        """
        print("\n" + "-" * 60)
        print("[阶段1] 初始化阶段")
        print("-" * 60)
        
        # 升降台复位到安全高度0.2m
        print("  升降台调整到安全高度0.2m")
        self.lift_client.set_height(height=0.2, speed=0.25)
        time.sleep(1.5)
        
        # 重置所有状态标志
        print("  重置状态标志")
        self.reset_position_location_barcode_value_state()
        self.is_real_drop = False
        
        # 导航到抓取点
        print("  导航到抓取点")
        self.robot_move(self.position_catch)
        time.sleep(2)
        
        print("  初始化完成")

    def stage_2_apriltag_detection(self):
        """
        阶段2: AprilTag识别
        - 调整位置到检测点
        - 降低升降台高度
        - 扫描并保存偏移量
        """
        print("\n" + "-" * 60)
        print("[阶段2] AprilTag识别")
        print("-" * 60)

        # 位置调整
        ahead_cfg = self.apriltag_scan_steps.get('ahead', {})
        left_cfg = self.apriltag_scan_steps.get('left', {})
        back_cfg = self.apriltag_scan_steps.get('back', {})

        print("  根据配置调整检测位置")
        print(f"    向前: {ahead_cfg.get('distance', 0):.2f}m, 向左: {left_cfg.get('distance', 0):.2f}m, 后退: {back_cfg.get('distance', 0):.2f}m")

        self.amr.robot_step_control(direction='ahead',
                                    distance=ahead_cfg.get('distance', 0),
                                    speed=ahead_cfg.get('speed', self.step_speed))
        time.sleep(ahead_cfg.get('distance', 0)/ahead_cfg.get('speed', self.step_speed)+1)

        self.amr.robot_step_control(direction='left',
                                    distance=left_cfg.get('distance', 0),
                                    speed=left_cfg.get('speed', self.step_speed))
        time.sleep(left_cfg.get('distance', 0)/left_cfg.get('speed', self.step_speed)+2)

        self.amr.robot_step_control(direction='back',
                                    distance=back_cfg.get('distance', 0),
                                    speed=back_cfg.get('speed', self.step_speed))
        time.sleep(back_cfg.get('distance', 0)/back_cfg.get('speed', self.step_speed)+1)

        # 降低升降台高度
        print("  升降台调整到0.00m（检测高度）")
        self.lift_client.set_height(height=0.00, speed=0.25)
        time.sleep(1.5)

        # 机械臂调整到检测姿态
        print("  机械臂调整到检测姿态")
        target_poses = [
            [1, [0,0,0,-90,0,0,0,0,0,0,-90,0,0,0]],
        ]
        execute_joint_trajectory(self.robot_sdk, target_poses)
        time.sleep(1)
        
        # 头部摄像头复位
        print("  头部摄像头复位")
        self.head_publisher(vertical_angle=0, horizontal_angle=0)

        # 右臂下放到扫描位置
        print("  右臂调整到扫描位置（-120度）")
        right_arm_down = [0, 0, 0, -120, 0, 0, 0]
        self.arm_publisher([0,0,0,-90,0,0,0], right_arm_down, times=[1])
        time.sleep(1.5)

        # 扫描AprilTag
        print("  开始扫描AprilTag...")
        tag_info = self.get_aprtag()
        
        horizontal_offset = float(tag_info['off_horizontal'])
        vertical_offset = float(tag_info['off_vertical'])
        off_camera = float(tag_info['off_camera'])
        
        # 保存偏移量供后续使用
        self.apriltag_offsets = {
            'horizontal_offset': horizontal_offset,
            'vertical_offset': vertical_offset,
            'off_camera': off_camera
        }
        
        print(f"  AprilTag识别结果:")
        print(f"    水平偏移: {horizontal_offset:.3f}m")
        print(f"    垂直偏移: {vertical_offset:.3f}m")
        print(f"    相机距离: {off_camera:.3f}m")

        # 升降台回到0.2m
        print("  升降台调整到0.2m（为条码扫描准备）")
        self.lift_client.set_height(height=0.2, speed=0.25)
        time.sleep(1.5)

        print("  AprilTag识别完成")

        return self.apriltag_offsets

    def stage_3_barcode_scanning(self):
        """
        阶段3: 条形码扫描
        - 移动到扫码位置
        - 调整升降台高度
        - 扫描条码并等待MES消息
        """
        print("\n" + "-" * 60)
        print("[阶段3] 条形码扫描")
        print("-" * 60)
        
        # 向前移动到扫码位置（回到抓取点+扫码偏移）
        ahead_cfg = self.barcode_scan_steps.get('ahead', {})
        ahead_distance = ahead_cfg.get('distance', 0)
        ahead_speed = ahead_cfg.get('speed', 0.8)
        total_ahead = 0.1 + ahead_distance
        print(f"  向前移动 {total_ahead:.2f}m (回到抓取点0.1m + 扫码偏移{ahead_distance:.2f}m)")
        self.amr.robot_step_control(direction='ahead', distance=total_ahead, speed=ahead_speed)
        time.sleep(total_ahead/ahead_speed + 1)
        
        # 向右移动对准条码
        right_cfg = self.barcode_scan_steps.get('right', {})
        right_distance = right_cfg.get('distance', 0)
        right_speed = right_cfg.get('speed', self.step_speed)
        print(f"  向右移动 {right_distance:.2f}m (对准条码)")
        self.amr.robot_step_control(direction='right', distance=right_distance, speed=right_speed)
        time.sleep(right_distance/right_speed + 1)
        
        # 升降台调整到扫码高度
        barcode_height = self.barcode_scan_height
        print(f"  升降台调整到{barcode_height:.2f}m（扫码高度）")
        self.lift_client.set_height(height=barcode_height, speed=0.25)
        time.sleep(1)
        
        # 重置条码状态
        self.reset_barcode_state()
        
        # 启动条码检测
        print("  启动条码检测...")
        while not rospy.is_shutdown():
            arm_for_detector_code(self.robot_sdk, start_time=0.5, time_duration=0.6, mode=0)
            
            if self.barcode_received:
                detected_barcode = self.latest_barcode_data
                print(f"  检测到条码: {detected_barcode}")
                self.reset_barcode_state()
                break
        
        # 等待MES消息
        print("  扫码完成，等待MES系统消息...")
        mes_msg = self._wait_for_position1()

        if mes_msg is None:
            print("  未收到MES消息，任务终止")
            return None, None

        print("  条形码扫描完成，已收到MES消息")

        return detected_barcode, mes_msg

    def stage_5_grab_box(self, tag_offsets):
        """
        阶段5: 抓取箱子
        - 使用AprilTag偏移量调整位置
        - 执行抓取动作
        - 检测箱子重量
        """
        print("\n" + "-" * 60)
        print("[阶段5] 抓取箱子")
        print("-" * 60)
        
        horizontal_offset = tag_offsets['horizontal_offset']
        vertical_offset = tag_offsets['vertical_offset']
        
        print(f"  使用AprilTag偏移量: 水平{horizontal_offset:.3f}m, 垂直{vertical_offset:.3f}m")
        
        # 机械臂准备抓取姿态
        print("  机械臂调整到抓取准备姿态")
        self.prepare_catch_box()
        
        # 向右移动调整位置
        catching_cfg = self.robot_params.get('catching', {})
        right_adjust_distance = catching_cfg.get('right_adjust_distance', 0.05)
        print(f"  向右移动 {right_adjust_distance:.2f}m（位置调整）")
        self.amr.robot_step_control(direction='right', distance=right_adjust_distance, speed=self.step_speed)
        time.sleep(right_adjust_distance/self.step_speed + 2)
        
        # 升降台调整到抓取高度
        catch_height = self.catch_height
        print(f"  升降台调整到{catch_height:.2f}m（抓取高度）")
        self.lift_client.set_height(height=catch_height, speed=0.25)
        
        # 等待动作完成
        time_sleep_horizontal = right_adjust_distance/self.step_speed + 3
        time_sleep_lift = max(self.height_speed_reciprocal * catch_height, 1.5)
        wait_time = max(time_sleep_horizontal, time_sleep_lift)
        print(f"  等待动作完成 {wait_time:.1f}秒")
        time.sleep(wait_time)
        
        # 向前推进到箱体位置
        x_distance = self.catch_x_distance
        catching_cfg = self.robot_params.get('catching', {})
        use_fixed_advance = catching_cfg.get('use_fixed_advance', False)

        if use_fixed_advance:
            catch_ahead_distance = catching_cfg.get('fixed_advance_distance', 1.2)
            print(f"  向前推进 {catch_ahead_distance:.2f}m（固定距离模式）")
        else:
            catch_ahead_distance = round(abs(vertical_offset - x_distance), 2)
            print(f"  向前推进 {catch_ahead_distance:.2f}m（计算距离模式）")

        self.amr.robot_step_control(direction='ahead', distance=catch_ahead_distance, speed=0.8)
        # self.amr.robot_step_control(direction='ahead', distance=catch_ahead_distance, speed=0.8, avoid=0)
        time.sleep(catch_ahead_distance/0.8 + 1.5)

        # 执行抓取动作
        print("  执行抓取动作")
        self.catch_box("height_1")
        time.sleep(1.5)
        
        # 检查抓取状态并检测重量
        print("  检查抓取状态...")
        if self.motor_current_received and self.latest_motor_current:
            current_12 = self.latest_motor_current[12]
            current_13 = self.latest_motor_current[13]
            print(f"    电流12: {current_12:.3f}, 电流13: {current_13:.3f}")
            
            if abs(current_12) < self.drop_current_threshold_12 and abs(current_13) < self.drop_current_threshold_13:
                print("    警告：电流异常偏低，可能未成功抓取！")
            else:
                print("    电流正常，抓取成功")
            
            # 检测重量
            time.sleep(0.3)
            self.detected_weight_level = self._detect_box_weight()
            if self.detected_weight_level:
                weight_cn = {'light': '轻', 'medium': '中', 'heavy': '重'}[self.detected_weight_level]
                print(f"    重量检测结果: {weight_cn}")
        else:
            print("    未收到电流数据")
            self.detected_weight_level = None
        
        # 抬升离开抓取点
        print(f"  抬升 {self.catched_leave_height:.2f}m（离开抓取点）")
        self.amr.robot_lift_control(direction="up", height=self.catched_leave_height)
        time.sleep(self.height_speed_reciprocal * self.catched_leave_height + 2)
        
        print("  抓取完成")

    def stage_6_transport_box(self, mes_msg):
        """
        阶段6: 搬运箱子
        - 启动掉箱检测
        - 后退离开抓取区
        - 导航到放置位置
        """
        print("\n" + "-" * 60)
        print("[阶段6] 搬运箱子")
        print("-" * 60)
        
        # 启动掉箱检测
        print("  启动掉箱检测线程")
        print(f"    当前电流状态: {'已接收' if self.motor_current_received else '未接收'}")
        if self.motor_current_received and self.latest_motor_current:
            print(f"    当前电流13: {self.latest_motor_current[13]:.3f}")
        
        self.box_carrying = True
        drop_thread = threading.Thread(target=self._drop_monitor_loop, daemon=True)
        drop_thread.start()
        time.sleep(0.3)
        
        # 后退离开抓取区域
        print(f"  后退 {self.step_back_distance:.2f}m（离开抓取区）")
        self.back_step()

        # 升降台调整到搬运高度
        print(f"  升降台调整到{self.normal_travel_height:.2f}m（搬运高度）")
        self.lift_client.set_height(height=self.normal_travel_height, speed=0.25)
        
        time.sleep(1)
        # 导航到放置位置 - 直接使用MES返回的坐标（与旧代码保持一致）
        pos = mes_msg["position"]
        loc = mes_msg["location"]

        print(f"最终接收到：位置={pos}, 库位号={loc}")

        self.robot_move(pos)
        
        # 头部复位
        self.head_publisher(vertical_angle=0, horizontal_angle=0)
        
        print("  搬运完成")

    def stage_7_place_box(self, mes_msg):
        """
        阶段7: 放置箱子
        - 关闭掉箱检测
        - 根据库位确定放置层级（支持B货架左右列分离）
        - 调整高度并释放箱子
        """
        self.reload_config()
        print("\n" + "-" * 60)
        print("[阶段7] 放置箱子")
        print("-" * 60)
        
        loc = mes_msg["location"]
        
        # 关闭掉箱检测
        print("  关闭掉箱检测（放置阶段电流降低为正常现象）")
        self.box_carrying = False
        self.is_real_drop = False
        time.sleep(0.2)
        
        # 确定货架类型和层级
        print(f"  解析库位号 '{loc}'")

        # 根据库位号前缀确定货架类型
        if loc.startswith('A'):
            rack_type = 'rack_A'
            rack_name = 'A货架'
        elif loc.startswith('B'):
            # B货架需要区分左右列
            # B1, B3, B5 是左列（奇数）
            # B2, B4, B6 是右列（偶数）
            if loc in ['B1', 'B3', 'B5']:
                rack_type = 'rack_B_left'
                rack_name = 'B货架-左列'
            elif loc in ['B2', 'B4', 'B6']:
                rack_type = 'rack_B_right'
                rack_name = 'B货架-右列'
            else:
                print(f"  未知B货架库位号: {loc}")
                return
        else:
            print(f"  未知库位号格式: {loc}")
            return

        # 获取货架专用配置，如果没有则使用通用配置
        rack_config = self.placement_layers.get(rack_type, {})
        default_config = self.placement_layers

        print(f"    货架类型: {rack_name}")
        print(f"    rack_type: {rack_type}")
        print(f"    rack_config 存在: {bool(rack_config)}")

        # 确定层级
        first_layer = ["A1", "A2", "B1", "B2"]
        second_layer = ["A3", "A4", "B3", "B4"]
        third_layer = ["A5", "A6", "B5", "B6"]

        if loc in first_layer:
            layer_name = "第一层"
            layer_config_key = 'first_layer'
            layer_config = rack_config.get(layer_config_key, default_config.get(layer_config_key, {}))
            place_type = "height_2"

            arm_action_fang = {
                'left_arm_traj': [-55, -15, 0, -45, -5, -3, 0],
                'right_arm_traj': [-55, 15, 0, -45, 5, -3, 0]
            }

        elif loc in second_layer:
            layer_name = "第二层"
            layer_config_key = 'second_layer'
            layer_config = rack_config.get(layer_config_key, default_config.get(layer_config_key, {}))
            place_type = "height_3"

            arm_action_fang = {
                'left_arm_traj': [-73, -15, 0, -27, -5, -3, 0],
                'right_arm_traj': [-73, 15, 0, -27, 5, -3, 0]
            }

        elif loc in third_layer:
            layer_name = "第三层"
            layer_config_key = 'third_layer'
            layer_config = rack_config.get(layer_config_key, default_config.get(layer_config_key, {}))
            place_type = "height_4"

            arm_action_fang = {
                'left_arm_traj': [-125, -13, -5, -10, 0, 0, 35],
                'right_arm_traj': [-125, 13, 5, -10, 0, 0, 35]
            }

        else:
            print(f"  未知库位号: {loc}")
            return

        # 获取层级参数
        target_height = layer_config.get('target_height', 0.33)
        place_ahead_distance = layer_config.get('ahead_distance', 0.8)
        extra_wait_time = layer_config.get('extra_wait_time', 5.0)

        print(f"    层级: {layer_name}")
        print(f"    最终参数:")
        print(f"      高度: {target_height:.2f}m")
        print(f"      推进距离: {place_ahead_distance:.2f}m")
        print(f"      额外等待: {extra_wait_time:.1f}秒")
        
        # 机械臂调整到放置姿态
        print(f"  机械臂调整到{layer_name}放置姿态")
        self.arm_publisher(arm_action_fang['left_arm_traj'], arm_action_fang['right_arm_traj'], times=self.arm_action_times)
        time.sleep(self.arm_action_times[0] + 1.5)
        
        # 根据重量调整高度
        print("  根据箱子重量微调放置高度")
        time.sleep(0.5)
        original_height = target_height
        weight_level = self._detect_box_weight()
        adjusted_height = self._adjust_height_by_weight(base_height=original_height, weight_level=weight_level)
        
        # 升降台调整到放置高度
        target_height = adjusted_height
        print(f"  升降台调整到{target_height:.2f}m（放置高度）")
        self.lift_client.set_height(height=target_height, speed=0.25)
        time.sleep(2)
        
        # 向前推进放入库位
        print(f"  向前推进 {place_ahead_distance:.2f}m（放入库位）")
        self.amr.robot_step_control(direction='ahead', distance=place_ahead_distance, speed=0.8, avoid=1)
        time.sleep(place_ahead_distance/self.step_speed + extra_wait_time)
        
        # 释放箱子
        print("  执行释放动作")
        self.release_box(place_type)
        
        # 后退离开库位
        print(f"  后退 {self.step_back_distance:.2f}m（离开库位）")
        self.back_step()
        
        print("  放置完成")

    def stage_8_task_completion(self):
        """
        阶段8: 任务完成
        - 重置状态
        - 机械臂/升降台复位
        - 设置任务成功状态
        """
        print("\n" + "-" * 60)
        print("[阶段8] 任务完成")
        print("-" * 60)
        
        # 重置状态
        print("  重置状态标志")
        self.reset_position_location_barcode_value_state()
        
        # 升降台复位
        print(f"  升降台调整到{self.normal_travel_height:.2f}m（搬运高度）")
        self.lift_client.set_height(height=self.normal_travel_height, speed=0.25)
        time.sleep(1)
        
        # 机械臂复位
        print("  机械臂复位到初始姿态")
        self.arm_back()
        time.sleep(2)
        
        # 设置任务状态
        print("  设置任务状态为成功")
        if rospy.has_param('/task_status'):
            rospy.delete_param('/task_status')
        try:
            rospy.set_param("/task_status", "success")
            print("  任务状态设置成功")
        except Exception as e:
            print(f"  任务状态设置失败: {e}")
        
        print("  单次任务完成")

    # ------------------------------
    # 主任务流程
    # ------------------------------

    def process_task(self):
        """单次任务主流程"""
        print("\n" + "-" * 60)
        print("开始执行单次任务")
        print("-" * 60)
        
        # 阶段1: 初始化
        self.stage_1_initialization()
        
        # 阶段2: AprilTag识别
        tag_offsets = self.stage_2_apriltag_detection()
        
        # 阶段3: 条形码扫描
        result = self.stage_3_barcode_scanning()
        if result is None or result[0] is None:
            print("任务中止：条码扫描失败")
            return
        barcode, mes_msg = result
        
        # 阶段5: 抓取箱子
        self.stage_5_grab_box(tag_offsets)
        
        # 阶段6: 搬运箱子
        self.stage_6_transport_box(mes_msg)
        
        # 阶段7: 放置箱子
        self.stage_7_place_box(mes_msg)
        
        # 阶段8: 任务完成
        self.stage_8_task_completion()
        
        print("\n" + "-" * 60)
        print("单次任务执行完成")
        print("-" * 60)

    def task_init(self):
        """任务全局初始化"""
        print("-" * 60)
        print("系统初始化")
        print("-" * 60)
        self.arm_back()
        time.sleep(1)
        self.head_publisher(vertical_angle=0, horizontal_angle=0)
        print("初始化完成")

    def run_tasks(self):
        """批量执行任务 - 带配置热重载"""
        self.task_init()
        
        for task_num in range(self.task_repeat):
            if self.task_state == 'stopped':
                print(f"任务在第 {task_num+1}/{self.task_repeat} 次执行时停止")
                return
            
            # ✨ 核心改动：每次任务前重载配置
            if task_num > 0:  # 第一次初始化时已加载，后续任务前重载
                print(f"\n{'='*60}")
                print(f"第 {task_num+1} 次任务前重新加载配置文件")
                print(f"{'='*60}")
                success = self.reload_config()
                if not success:
                    print("⚠️ 配置重载失败，使用旧配置继续")
                time.sleep(1)  # 给一点时间看日志
            
            print(f"\n开始执行第 {task_num+1}/{self.task_repeat} 次任务")
            self.process_task()
        
        print(f"\n所有任务执行完成！共执行 {self.task_repeat} 次")
        self.task_state = 'stopped'

def main():
    """程序主入口"""
    demo = BoxPickPlaceDemo(ws_addr="ws://169.254.128.2:5480/")
    robot_arm = KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    demo._cb_add_subscriptions()
    signal.signal(signal.SIGINT, demo._cb_signal_handler)
    demo_ping = threading.Thread(target=demo._cb_send_as_ping, daemon=True)
    demo_ping.start()
    demo.run_tasks()


if __name__ == '__main__':
    main()