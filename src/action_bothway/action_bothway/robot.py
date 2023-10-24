import threading
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from bothway_interfaces.action import AskNbReplyRobot
from bothway_interfaces.action import AskRobotReplyNb
# 导入Action接口

# 两个线程，一个Main线程，一个定时任务线程
class Robot(Node):
    """Action客户端"""

    def __init__(self, name):
        super().__init__(name)
        current_thread = threading.current_thread()
        self.get_logger().info(
            f"Main thread: {current_thread.name}, ID: {current_thread.ident}")
        self.send_index = 0
        self.get_logger().info(f"节点已启动：{name}!")
       
        self.action_server_reply_robot = ActionServer(
            self,
            AskRobotReplyNb,
            "AskRobotReplyNb",
            execute_callback=self.callback_execute_callback,# 主线程运行
        )
        self.timer = threading.Timer(2, self.send_request)
        self.timer.start()
    def callback_execute_callback(self,goal_handle: ServerGoalHandle):
        current_thread = threading.current_thread()
        self.get_logger().info(
            f"callback_execute_callback thread: {current_thread.name}, ID: {current_thread.ident}")
        playwave = goal_handle.request.base64wav
        self.get_logger().info(f"成功拿到播放参数：{playwave}")
        self.get_logger().info("播放成功")
        #给反馈
        goal_handle.succeed()
        result = AskRobotReplyNb.Result()
        result.code = 0
        result.msg = '播放成功'
        result.t = time.time()
        return result
    def send_request(self):
        #因为启动了定时器，所以是在子线程中运行
        current_thread = threading.current_thread()
        self.get_logger().info(
            f"request thread: {current_thread.name}, ID: {current_thread.ident}")
        # 每次创建一个新的client
        # test = rclpy.create_node("test")
        action_client_ask_nb = ActionClient(
            self, AskNbReplyRobot, "ask_nb_reply_robot")
        
        while not action_client_ask_nb.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        goal_msg = AskNbReplyRobot.Goal()
        goal_msg.base64wav = f"base64wav--aaaaa:{self.send_request}"

        goal_msg.index = self.send_index
        self.send_index = self.send_index + 1
        goal_msg.t = time.time()
        self.get_logger().info(f"goal_msg封装完成:{goal_msg.t}，准备发送请求")
        
        _send_goal_future = action_client_ask_nb.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info("等待 send_goal_future返回中.....")

        # rclpy.spin_until_future_complete(self, _send_goal_future)
        while not _send_goal_future.done(): # 用于阻塞当前线程，当前线程中其他代码都不能执行
            time.sleep(0.2)
            self.get_logger().info("等待 send_goal_future返回中.....")

        self.get_logger().info("请求成功返回")

        goal_handle = _send_goal_future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            self.get_logger().info('等待result_future返回中......')
            # rclpy.spin_until_future_complete(self, result_future)
            while not result_future.done():
                time.sleep(0.2)
                self.get_logger().info("等待 result_future.....")
            self.get_logger().info('result_future成功')

        if result_future == None or result_future.result() == None or result_future.result().result == None:
            self.get_logger().info(
                "result_future == None or result_future.result() == None or result_future.result().result == None")
            return None
        self.get_logger().info(f"result_future:{result_future}")
        self.get_logger().info(
            f"result_future.result():{result_future.result()}")
        
        self.timer = threading.Timer(0.1, self.send_request)
        self.timer.start()
        
    def feedback_callback(self, feedback_msg):  # 此处是第一个话题，过程反馈话题，客户端订阅
        """获取回调反馈"""
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Received feedback: {feedback.state}, {feedback.process}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    robot = Robot("robot")
    # robot.send_request()
    rclpy.spin(robot)
    rclpy.shutdown()
