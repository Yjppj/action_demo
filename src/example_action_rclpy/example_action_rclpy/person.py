import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# 导入Action接口

from robot_control_interfaces.action import AskControlReplyPerson
class ActionClientAskControl(Node):
    """Action客户端"""
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        self.index = 0
        self.send_goal_timer_ = self.create_timer(3, self.request)
    
    def request(self): # 此处创建第一个服务：目标传递服务
        """发送目标"""    
        # self.send_goal_timer_.cancel()
        goal_msg = AskControlReplyPerson.Goal()
        goal_msg.order = "移动"
        goal_msg.index = self.index 
        self.index = self.index + 1
        # 连接服务端（机器人）
        action_client_ask_control = ActionClient(self,AskControlReplyPerson,"ask_control_reply_person")
        self.get_logger().info(f" Person 发送请求开始，client:{action_client_ask_control}")
        while not action_client_ask_control.wait_for_server(timeout_sec=1):
            self.get_logger().info("person wait_for_ control server....")
            time.sleep(1)
        # 发送请求，如果有反馈，会自动调用feedback_callback，这样的话，可以一直获取到机器人的状态，_send_goal_future是服务的响应
        
        _send_goal_future = action_client_ask_control.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.get_logger().info("等待 send_goal_future返回中.....")
        rclpy.spin_until_future_complete(action_client_ask_control, _send_goal_future, timeout_sec=10)
        self.get_logger().info("请求成功返回")
        result_future =  _send_goal_future.result().get_result_async()
        self.get_logger().info('等待result_future返回中......')
        rclpy.spin_until_future_complete(action_client_ask_control, result_future, timeout_sec=10)
        self.get_logger().info(
            f"result_future.result():{result_future.result()}")
        action_client_ask_control.destroy_node()
        return result_future.result().result


    def feedback_callback(self,feedback_msg):  # 此处是第一个话题，过程反馈话题，客户端订阅
        """获取回调反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.pose}")

    def goal_response_callback(self,future):
        """收到目标处理结果"""
        goal_handle = future.result()
        # 告知client 收到了目标， 此处是第一个服务：目标传递服务 ， 此处是客户端（机器人） 给的回复
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected ：(")
            return
        self.get_logger().info('Goal accepted :)')
        # 这行代码也是一个服务，是第二个服务：结果传递服务，用来获取服务端（server）执行的结果情况，这行代码执行之后没有反馈 
        # 此行代码执行完成之后，feedback_callback持续多次执行，当feedback_callback执行完成之后，等于说机器人执行目标完成了，然后开始执行下一行
        self._get_result_future = goal_handle.get_result_async()
        # 这行代码会拿到机器人任务执行完成之后的结果，此处的get_result_callback不是通过话题获取的
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """获取结果反馈"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.pose}')

        
        


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    Person = ActionClientAskControl("action_client_ask_control")  
    rclpy.spin(Person)
    rclpy.shutdown()
