import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
# 导入接口
from robot_control_interfaces.action import MoveRobot
# 导入机器人类
from robot_control_interfaces.action import MoveControl
from example_action_rclpy.robot import Robot


class ActionRobotAsServer(Node):
    """机器人端Action服务"""
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"节点已经启动：{name}")
        self.robot_ = Robot()
        # 创建一个action server 
        self.action_server_ = ActionServer(
            self, MoveRobot, "move_robot", self.execute_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info("执行移动机器人")
        
        #机器人执行自己的方法，运行起来，参数来源于client端，此处是第一个服务目标传递服务的接收
        for i in range(10):
            feedback_msg = MoveRobot.Feedback()
            print(i)
            feedback_msg.pose = f" 第{goal_handle.request.index} 次命令 机器人 第{i}次移动  "
            feedback_msg.status = 1
            # 此处是过程反馈话题，机器人server端移动的时候，不断的往这个topic发送消息
            goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result = MoveRobot.Result()
        result.pose = f"第{goal_handle.request.index} 次命令机器人移动完毕"
        return result


      
 
def main(args=None):
    """主函数"""

    rclpy.init(args=args)
    action_robot_as_server = ActionRobotAsServer("action_robot_as_server")
    rclpy.spin(action_robot_as_server)
    rclpy.shutdown()
