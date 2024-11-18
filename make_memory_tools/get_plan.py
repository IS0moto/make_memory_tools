
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class GetPlan(Node):
    def __init__(self):
        super().__init__('getPlanNode')

        self.sub_local_plan = self.create_subscription(
            Path,
            '/local_plan',
            self.cb_local_plan,
            10
        )

        self.sub_plan = self.create_subscription(
            Path,
            '/plan',
            self.cb_plan,
            10
        )

        self.controll_loop = self.create_timer(
            0.1,
            self.run
        )

    
    def cb_local_plan(self, msg):
        # self.get_logger().info("Update local plan")
        pass

    def cb_plan(self, msg:Path):
        self.get_logger().info(f"{len(msg.poses)}")


    def run(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    get_plan_node = GetPlan()
    try:
        rclpy.spin(get_plan_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()