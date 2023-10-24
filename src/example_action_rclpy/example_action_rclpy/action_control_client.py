import base64
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
# 导入Action接口
from robot_control_interfaces.action import MoveRobot

from robot_control_interfaces.action import AskControlReplyPerson
class ActionControlAsClient(Node):
    """Action客户端"""
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        self.index = 0
        self.send_goal_timer_ = self.create_timer(3, self.request)
        self.action_server_reply_person = ActionServer(
            self,
            AskControlReplyPerson,
            "ask_control_reply_person",
            self.callback_reply_person,
        )
    def callback_reply_person(self, goal_handle: ServerGoalHandle):
        
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info(f"=========== START execute callback_ask_control_reply_person INDEX {goal_handle.request.index} ===========")
        start_time = time.time()
        # 给client的反馈，我们暂时不需要
        feedback_msg = AskControlReplyPerson.Feedback()
        # try:
        # base64库对音频数据进行解码
        t = time.time()
        self.get_logger().info(f"nb receive robotmicdata , base64wav : {goal_handle.request.audio[:20]},cost time:{time.time() - goal_handle.request.t}")

        audio_data = base64.b64decode(goal_handle.request.audio)
        index = goal_handle.request.index
        # base_path = f'{os.getcwd()}/audios/reduce_noise'
        # if not os.path.exists(base_path):
        #     os.makedirs(base_path, exist_ok=True)
        # output_file = os.path.join(base_path, f'{index}.wav')
        # with open(output_file, "wb") as f:
        #     f.write(audio_data)
        # audio_data = self._audio_noise_reduce(audio_data)

        directions_list = goal_handle.request.directions_list
        self.save_mic_test_log(
            goal_handle.request.index, "1:client->server", t - goal_handle.request.t)
        # 对上面得到的数据进行转录
        t2 = time.time()




        # 下面是返回给机器人的
        goal_handle.succeed()
        result = AskControlReplyPerson.Result()
        result.code = 0
        return result
    def request(self): # 此处创建第一个服务：目标传递服务
        """发送目标"""  
        
        # self.send_goal_timer_.cancel()
        goal_msg = MoveRobot.Goal()
        goal_msg.distance = 5.0
        goal_msg.index = self.index 
        self.index = self.index + 1
        # 连接服务端（机器人）
        action_client_ask_robot = ActionClient(self,MoveRobot,"move_robot")
        self.get_logger().info(f"发送请求开始，client:{action_client_ask_robot}")
        while not action_client_ask_robot.wait_for_server(timeout_sec=1):
            self.get_logger().info("control wait_for_ robot server....")
            time.sleep(1)
        _send_goal_future = action_client_ask_robot.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.get_logger().info("等待 send_goal_future返回中.....")
        rclpy.spin_until_future_complete(action_client_ask_robot, _send_goal_future, timeout_sec=10)
        self.get_logger().info("请求成功返回")

        result_future =  _send_goal_future.result().get_result_async()
        self.get_logger().info('等待result_future返回中......')
        rclpy.spin_until_future_complete(action_client_ask_robot, result_future, timeout_sec=10)
        self.get_logger().info(
            f"result_future.result():{result_future.result()}")
        action_client_ask_robot.destroy_node()
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
    action_control_as_client = ActionControlAsClient("action_control_as_client")  
    rclpy.spin(action_control_as_client)
    rclpy.shutdown()
