import sys
import time

import rclpy
from rclpy.node import Node
from iot_project_interfaces.msg import TargetsTimeLeft
from iot_project_interfaces.srv import TaskAssignment
from iot_project_interfaces.srv import ColorTarget


from geometry_msgs.msg import Point, Vector3
from ros_gz_interfaces.srv import ControlWorld

from threading import Thread

from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry


from iot_project_manager.math_utils import *
import iot_project.sim_utils as sim_utils


TARGET_EPS = 0.8
TIME_BEFORE_PLAY = 10
GAZEBO_WORLD = "iot_project_world"


class TargetHandler(Node):

    def __init__(self, sim_config = None):
        super().__init__('target_publisher')
        
        self.sim_config : sim_utils.SimulationConfig = sim_config

        self.clock = 0
        
        self.targets = []
        self.targets_state_has_changed = []
        self.drone_odometry_topics = []


        self.drone_positions = {}
        self.drones_last_visit = {}

        self.set_target_service = self.create_service(
            TaskAssignment,
            'task_assigner/set_task',
            self.set_task
        )
        
        self.get_task_service = self.create_service(
            TaskAssignment,
            'task_assigner/get_task',
            self.announce_task
        )


        self.targets_time_publisher = self.create_publisher(
            TargetsTimeLeft,
            'task_assigner/targets_time_left',
            10
        )

        self.time_subscriber = self.create_subscription(
            Clock,
            '/world/%s/clock' % GAZEBO_WORLD,
            self.store_time_callback,
            10
        )

        self.color_changer_service = self.create_client(
            ColorTarget,
            'iot_animator/change_color'
        )


        self.world_client = self.create_client(
            ControlWorld,
            '/world/%s/control' % GAZEBO_WORLD
        )


        for d in range(self.sim_config.no_drones):

            self.create_subscription(
                Odometry,
                "/X3_%d/odometry" % d,
                lambda msg, drone="X3_%d"%d: self.register_drone_position(msg, drone),
                10,
            )

        
        self.get_logger().info("Target handler started. Task can be retrieved from service /task_assigner/get_task")

        self.create_timer(0.1, self.publish_targets_time)
        self.simulation_start_timer = self.create_timer(TIME_BEFORE_PLAY, self.start_simulation)

        Thread(target=self.start_tester).start()


    def start_simulation(self):

        self.world_client.wait_for_service()

        msg = ControlWorld.Request()
        msg.world_control.pause = False
        
        self.world_client.call_async(msg)
        self.simulation_start_timer.destroy()

        self.get_logger().info("Simulation will now play!")


    def start_tester(self):

        self.get_logger().info("Starting tester")

        while True:
            for t in range(len(self.targets)):
                
                currently_visited = False

                for d in self.drone_positions.keys():

                    p0 = (self.drone_positions[d].x, self.drone_positions[d].y, self.drone_positions[d].z)
                    p1 = (self.targets[t]["position"].x, self.targets[t]["position"].y, self.targets[t]["position"].z)
                    

                    if point_distance(p0, p1) < TARGET_EPS:
                        self.drones_last_visit[d] = t
                        self.targets[t]["last_visit"] = float(self.clock)
                        self.targets[t]["expired"] = False
                        currently_visited = True

                if currently_visited != self.targets[t]["currently_visited"]:
                    if currently_visited:
                        self.change_color(self.targets[t]["name"], (0.0, 1.0, 0.0))
                    else:
                        self.change_color(self.targets[t]["name"], (0.0, 0.0, 1.0))

                if not self.targets[t]["expired"] and not self.targets[t]["currently_visited"] and self.targets[t]["last_visit"] + self.targets[t]["expiration_time"]*(10**9) < float(self.clock):
                    self.targets[t]["expired"] = True
                    self.change_color(self.targets[t]["name"], (1.0, 0.0, 0.0))

                self.targets[t]["currently_visited"] = currently_visited

            #self.advertise_visiting_times()
            time.sleep(0.01)

    # def advertise_visiting_times(self):
    #     self.targets_publisher.publish()  

    def register_drone_position(self, msg : Odometry, drone : str):
        self.drone_positions[drone] = msg.pose.pose.position


    def store_time_callback(self, msg : Clock):
        self.clock = msg.clock.sec * 10**9 + msg.clock.nanosec


    def publish_targets_time(self):

        targets_time_left = TargetsTimeLeft()

        for target in self.targets:
            targets_time_left.times.append(target["expiration_time"] * 10 ** 9 - (self.clock - target["last_visit"]))

        self.targets_time_publisher.publish(targets_time_left)




    def announce_task(self, request : TaskAssignment.Request, response : TaskAssignment.Response):

        response = TaskAssignment.Response()

        response.simulation_name = self.sim_config.simulation_name
        response.simulation_time = self.sim_config.simulation_time
        response.no_drones = self.sim_config.no_drones       
        response.aoi_weight = self.sim_config.aoi_weight
        response.violation_weight = self.sim_config.violation_weight
        response.fairness_weight = self.sim_config.fairness_weight

        wv = self.sim_config.wind_vector
        response.wind_vector = Vector3(x=wv[0], y=wv[1], z=wv[2])

        response.target_positions   = [target['position'] for target in self.targets]
        response.last_visits        = [target['last_visit'] for target in self.targets]
        response.target_thresholds  = [target['expiration_time'] for target in self.targets]        

        return response
        
    def set_task(self, request, response):

        self.get_logger().info("Setting task by service has been disabled for safety reason. Task is now set only at runtime.")

        return response
    

    def set_targets_from_config_file(self, config = None):

        if not config:
            config = self.sim_config

        targets = []
        
        for t in range(0, config.no_targets):
            
            try:
                targets.append(
                    {
                        "name"              : "Target %d" % (t + 1),
                        "position"          : Point(
                                                x = float(config.target_positions[t][0]),
                                                y = float(config.target_positions[t][1]),
                                                z = float(config.target_positions[t][2]),
                                            ),
                        "expiration_time"   : float(config.threshold_values[t]),
                        "last_visit"        : 0.0,
                        "currently_visited" : False,
                        "expired"           : False
                    }
                )

            except:
                break

        self.targets = targets

    def change_color(self, target : str, color : tuple):

        if len(color) < 3:
            return
        
        request = ColorTarget.Request()
        request.target = target
        request.r = color[0]
        request.g = color[1]
        request.b = color[2]

        self.color_changer_service.call_async(request)

def main():
    rclpy.init()

    # We assume that the simulation file is always given, as this node is only started
    # from the launch file and should be started only there to set up the simulation
    # correctly
    sim_config = sim_utils.import_simulation_config(sys.argv[1])
    
    executor = MultiThreadedExecutor()
    target_publisher = TargetHandler(sim_config)
    target_publisher.set_targets_from_config_file()

    executor.add_node(target_publisher)
    executor.spin()

    executor.shutdown()
    target_publisher.destroy_node()

    rclpy.shutdown()