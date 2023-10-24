import threading
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from bothway_interfaces.action import AskNbReplyRobot
from bothway_interfaces.action import AskRobotReplyNb
# 导入Action接口

# 只有一个主线程，可以使用rclpy.spin_until_future_complete
class Robot(Node):
    """Action客户端"""

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        self.action_server_reply_robot = ActionServer(
            self,
            AskNbReplyRobot,
            "ask_nb_reply_robot",
            # 当有节点向该服务发送请求时，ROS2会自动调用该回调函数，并将请求的数据传递给它
            self.callback_reply_robot
        )
    def callback_reply_robot(self, goal_handle: ServerGoalHandle):
        current_thread = threading.current_thread()
        self.get_logger().info(
            f"callback_reply_robot thread: {current_thread.name}, ID: {current_thread.ident}")

        self.get_logger().info(
            f"=========== START execute callback_ask_nb_reply_robot INDEX {goal_handle.request.index} ===========")
        feedback_msg = AskNbReplyRobot.Feedback()
        req_base64wav = goal_handle.request.base64wav
        req_index = goal_handle.request.index
        self.get_logger().info(
            f'收到机器人发来的请求req_base64wav:{req_base64wav},req_i:{req_index}')
        # 拿到音频进行tts处理
        goal_msg = AskRobotReplyNb.Goal()
        goal_msg.base64wav = "hello hello!"
        goal_msg.index = goal_handle.request.index

        self.get_logger().info("tts合成语音后再次发送给机器人播放")

        client = rclpy.create_node("client")
        action_client_ask_robot = ActionClient(
            client, AskRobotReplyNb, "AskRobotReplyNb")
        _send_goal_future = action_client_ask_robot.send_goal_async(goal_msg)

        # while not _send_goal_future.done():
        #     time.sleep(0.2)
        #     self.get_logger().info("播放请求 等待 send_goal_future返回中.....")
       
        rclpy.spin_until_future_complete(client, _send_goal_future)  # 是一种非阻塞的等待方式，允许ROS节点继续执行其他任务
        self.get_logger().info("播放请求 成功返回")
        goal_handle1 = _send_goal_future.result()
        if goal_handle1.accepted:
            result_future = goal_handle1.get_result_async()
            self.get_logger().info('播放请求 等待result_future返回中......')
            rclpy.spin_until_future_complete(client, result_future)
            # while not result_future.done():
            #     # rclpy.spin_once(client)
            #     time.sleep(0.02)
            #     self.get_logger().info("播放请求 等待 send_goal_future返回中.....")
            self.get_logger().info('播放请求 result_future成功')

        if result_future == None or result_future.result() == None or result_future.result().result == None:
            self.get_logger().info(
                " 播放请求 result_future == None or result_future.result() == None or result_future.result().result == None")
            return None
        self.get_logger().info(f"播放请求 result_future:{result_future}")
        self.get_logger().info(
            f"播放请求 result_future.result():{result_future.result()}")

        goal_handle.succeed()
        result = AskNbReplyRobot.Result()
        result.code = 0
        result.msg = 'success'
        result.t = time.time()
        self.get_logger().info(
            f"=========== END ：execute callback_ask_nb_reply_robot INDEX {goal_handle.request.index} ===========")
        client.destroy_node()
        return result


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    server = Robot("server")
    rclpy.spin(server)
    rclpy.shutdown()
