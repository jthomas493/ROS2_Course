#!/usr/bin/env python3
import math
import random
import rclpy

from functools import partial
from rclpy.node import Node

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

from turtlesim.srv import Spawn
from turtlesim.srv import Kill


class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("turtle_spawner")

        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")

        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )
        self.spawn_turle_timer_ = self.create_timer(
            1.0 / self.spawn_frequency_, self.spawn_turtle
        )
        self.catch_turtle_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

    def callback_catch_turtle(self, request, response):
        self.call_turtle_kill(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_turtle(self):
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_turtle_spawner(name, x, y, theta)

    def call_turtle_spawner(self, name, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for turtlesim server ..")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_spawner, name=name, x=x, y=y, theta=theta))

    def callback_call_turtle_spawner(self, future, name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " has spawned")
                spawned_turtle = Turtle()
                spawned_turtle.name = response.name
                spawned_turtle.x = x
                spawned_turtle.y = y
                spawned_turtle.theta = theta
                self.alive_turtles_.append(spawned_turtle)
                self.publish_alive_turtles()

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def call_turtle_kill(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for turtle removal...")
            
        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_kill, name=name))
    
    def callback_call_turtle_kill(self, future, name):
        try:
            future.result()
            for(i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))                    


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
