import time
import random

from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from rosgraph_msgs.msg import Clock
from iot_project_interfaces.srv import TaskAssignment
from iot_project_solution_interfaces.action import PatrollingAction

# new imports
import numpy as np
from geometry_msgs.msg import Point
from iot_project_interfaces.msg import TargetsTimeLeft
from sklearn.cluster import KMeans

from nav_msgs.msg import Odometry
from iot_project_solution_src.math_utils import *
from .ant_colony import find_patrol_route


# declare constants
ALPHA = 0.1 # distance weigh: smaller alpha -> higher importance to have a small distance
BETA = 1.0  # aoi_bonus weight

class TaskAssigner(Node):

    def __init__(self):

        super().__init__('task_assigner')
            
        self.task = None
        self.no_drones = 0  
        self.targets = []
        self.thresholds = []

        self.action_servers = []
        self.current_tasks =  []
        self.idle = []

        self.sim_time = 0

        ###
        self.drone_pos = []
        self.targets_time_left = []
        self.clusters = []
        self.cluster_labels = []
        self.drone_curr_targets = []    # will be a list of lists
        self.targets_aoi = []
        self.targets_locks = []
        ###

        self.cluster_list = []

        self.task_announcer = self.create_client(
            TaskAssignment,
            '/task_assigner/get_task'
        )

        self.sim_time_topic = self.create_subscription(
            Clock,
            '/world/iot_project_world/clock',
            self.store_sim_time_callback,
            10
        )
    

    # Function used to wait for the current task. After receiving the task, it submits
    # to all the drone topics
    def get_task_and_subscribe_to_drones(self):

        self.get_logger().info("Task assigner has started. Waiting for task info")

        while not self.task_announcer.wait_for_service(timeout_sec = 1.0):
            time.sleep(0.5)

        self.get_logger().info("Task assigner is online. Requesting patrolling task")

        assignment_future = self.task_announcer.call_async(TaskAssignment.Request())
        assignment_future.add_done_callback(self.first_assignment_callback)


    # Callback used for when the patrolling task has been assigned for the first time.
    # It configures the task_assigner by saving some useful values from the response
    # (more are available for you to read and configure your algorithm, just check
    # the TaskAssignment.srv interface).
    # The full response is saved in self.task, so you can always use that to check
    # values you may have missed. Or just save them here by editing this function.
    # Once that is done, it creates a client for the action servers of all the drones
    def first_assignment_callback(self, assignment_future):

        task : TaskAssignment.Response = assignment_future.result()

        self.task = task
        self.no_drones = task.no_drones
        self.targets = task.target_positions
        self.thresholds = task.target_thresholds

        self.current_tasks = [None]*self.no_drones
        self.idle = [True] * self.no_drones

        # drone position at the start of the simulation
        self.drone_pos = [Point(x=0.0, y=(-self.no_drones + 1 + 2.0 * i) , z=0.0) for i in range(self.no_drones)]
        self.drone_curr_targets = [None] * self.no_drones
        self.targets_aoi = [0] * len(self.targets)
        self.targets_locks = np.array([False] * len(self.targets))

        # dictionary with paths to assign to drones
        self.patrol_routes = { drone_id : [] for drone_id in range(self.no_drones)}     # needed for ant colony
        ###
        
        ###
        # get all weights for final score
        self.violation_w = task.violation_weight
        self.fairness_w = task.fairness_weight
        self.aoi_w = task.aoi_weight

        # create a subscriber to the Odometry topic to retrive position for each drone
        for d in range(self.no_drones):
            self.odometry_topic = self.create_subscription(
                Odometry,
                '/X3_%d/odometry' % d,
                lambda msg, drone_id=d: self.store_position_callback(msg, drone_id),
                10
            )

        if self.no_drones < len(self.targets):
            # convert targets coords from Points to tuples
            targets_coords = np.array([(i.x, i.y, i.z) for i in self.targets]).astype(int)
            # compute clusters
            self.clusters = KMeans(n_clusters=task.no_drones,random_state=0, n_init='auto').fit(targets_coords)
            self.cluster_labels = self.clusters.labels_
        else:
            self.cluster_labels = [i for i in range(len(self.targets))]
            if self.no_drones > len(self.targets):
                self.cluster_labels += [None for i in range(self.no_drones - len(self.targets))]
        
        # create cluster matrix
        for d in range(self.no_drones):
            targets_cluster = np.array(self.targets)[self.cluster_labels == d]
            self.cluster_list.append(targets_cluster)
                
        # sort cluster matrix based on y coordinate of cluster centroids
        self.cluster_list.sort(key= lambda array: self.get_centroid(array)[1])
        
        self.targets_time_left_topic = self.create_subscription(
            TargetsTimeLeft,
            '/task_assigner/targets_time_left',
            self.store_targets_time_left_callback,
            10
        )

        # Now create a client for the action server of each drone (or target if drones > targets)
        for d in range(min(self.no_drones, len(self.targets))): 
            self.action_servers.append(
                ActionClient(
                    self,
                    PatrollingAction,
                    'X3_%d/patrol_targets' % d,
                )
            )

    # given a list of points it returns the centroid
    def get_centroid(self,point_list: list[Point]):
        x_components = [x.x for x in point_list]
        y_components = [x.y for x in point_list]
        z_components = [x.z for x in point_list]
        centroid = (np.mean(x_components),np.mean(y_components), np.mean(z_components))
        return centroid
    
    # This method starts on a separate thread an ever-going patrolling task, it does that
    # by checking the idle state value of every drone and submitting a new goal as soon as
    # that value goes back to True
    def keep_patrolling(self):

        def keep_patrolling_inner():
            while True:
                for d in range(self.no_drones):
                    if self.idle[d]:
                        ###
                        # assign cluster to drone
                        #targets_cluster = np.array(self.targets)[self.cluster_labels == d]
                        Thread(target=self.submit_task, args=(d,)).start()
                        ###
                time.sleep(0.1)

        Thread(target=keep_patrolling_inner).start()

    
    # Submits a patrol task to a single drone. Basic implementation just takes the array
    # of targets and shuffles it. Is up to you to improve this part and come up with your own
    # algorithm.
    # 
    # TIP: It is highly suggested to start working on a better scheduling of the targets from here.
    #      some drones may want to inspect only a portion of the nodes, other maybe more.
    #
    #      You may also implement a reactive solution which checks for the target violation
    #      continuously and schedules precise tasks at each step. For that, you can call again
    #      the task_announcer service to get an updated view of the targets' state; the last
    #      visit of each target can be read from the array last_visits in the service message.
    #      The simulation time is already stored in self.sim_time for you to use in case
    #      Times are all in nanoseconds.
    def submit_task(self, drone_id, targets_to_patrol=None):

        self.get_logger().info("Submitting task for drone X3_%s" % drone_id)
    
        while not self.action_servers[drone_id].wait_for_server(timeout_sec = 1.0):
            return

        self.idle[drone_id] = False

        ###
        self.targets_locks[self.drone_curr_targets[drone_id]] = False

        # if cluster is only one target, just assign it as target to patrol
        if self.cluster_list[drone_id].size == 1:
            targets_to_patrol = [self.cluster_list[drone_id][0]]
        # cluster has more element and we decide which strategy to adopt
        else:
            targets_to_patrol = []

            # TODO add condition based on number of targets in cluster 
            #      and in cluster average distance

            if len(self.cluster_list[drone_id]) < 20:
                targets_to_patrol = self.greedy_patrol(drone_id)
            else:
                targets_to_patrol = self.ant_patrol(self.get_global_target_index(drone_id), drone_id)    

        ###
        patrol_task =  PatrollingAction.Goal()
        patrol_task.targets = targets_to_patrol

        patrol_future = self.action_servers[drone_id].send_goal_async(patrol_task)

        # This is a new construct for you. Basically, callbacks have no way of receiving arguments except
        # for the future itself. We circumvent such problem by creating an inline lambda functon which stores
        # the additional arguments ad-hoc and then calls the actual callback function
        patrol_future.add_done_callback(lambda future, d = drone_id : self.patrol_submitted_callback(future, d))

    def get_global_target_index(self,drone_id: int) -> list[int]:
        '''
            Given a drone id, it returns a list of the global indexes of all elements of the associated cluster
        '''
        return [self.targets.index(i) for i in self.cluster_list[drone_id]]

    def greedy_patrol(self, drone_id):
        '''
            Returns the next target to visit in a greedy way
        '''
        target_priorities = []
        # for each global target id in the drone cluster
        for i in self.get_global_target_index(drone_id):
            # if it's not current target AND if it's not locked
            if  i != self.drone_curr_targets[drone_id] and not self.targets_locks[drone_id]:
                # get the priority
                priority = calculate_target_priority(self.drone_pos[drone_id], 
                                                     self.targets[i], 
                                                     self.targets_aoi[i], 
                                                     self.thresholds[i], 
                                                     self.aoi_w, 
                                                     self.violation_w,
                                                     alpha=ALPHA,
                                                     beta=BETA)
                # append 
                target_priorities.append((priority,i))
        #print("TARGET PRIORITY",target_priorities)
        # get target id with maximum priority, min gets the tuple with min priority
        chosen_target_idx = min(target_priorities,key= lambda x: x[0])[1]
        #chosen_target_idx = target_priorities[np.argmin(target_priorities[:,0])][1].astype(int)
        self.drone_curr_targets[drone_id] = [chosen_target_idx]
        self.targets_locks[chosen_target_idx] = True   # take target lock
        return [self.targets[chosen_target_idx]]    # target to patrol
    
    def ant_patrol(self, tar_prio_idx, drone_id):
        '''
            Computes the quasi optimal path using the ACA
        '''
        aois = np.array(self.targets_aoi)[tar_prio_idx]
        thresholds = np.array(self.thresholds)[tar_prio_idx]
        # compute a near-optimal path for the drone
        # we do this in advance to save time
        path = find_patrol_route(np.array(self.targets)[tar_prio_idx], aois, thresholds, self.aoi_w, self.violation_w, self.drone_pos[drone_id],alpha=ALPHA,beta=BETA,ant_count=32)
        # save the path for later
        self.patrol_routes[drone_id] = path
        # assign the previously saved path to the drone
        # because now the drone is free
        return self.patrol_routes[drone_id]
        


    # Callback used to verify if the action has been accepted.
    # If it did, prepares a callback for when the action gets completed
    def patrol_submitted_callback(self, future, drone_id):

        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info("Task has been refused by the action server")
            return
        
        result_future = goal_handle.get_result_async()

        # Lambda function as a callback, check the function before if you don't know what you are looking at
        result_future.add_done_callback(lambda future, d = drone_id : self.patrol_completed_callback(future, d))


    # Callback used to update the idle state of the drone when the action ends
    def patrol_completed_callback(self, future, drone_id):
        self.get_logger().info("Patrolling action for drone X3_%s has been completed. Drone is going idle" % drone_id)
        self.idle[drone_id] = True


    # Callback used to store simulation time
    def store_sim_time_callback(self, msg):
        self.clock = msg.clock.sec * 10**9 + msg.clock.nanosec

    # Callback used to store time left until expiration for each target
    def store_targets_time_left_callback(self, msg):
        self.targets_time_left = msg.times
        # temp = np.array(self.targets_time_left) / 10**9
        self.targets_aoi = np.array(self.thresholds) - np.array(self.targets_time_left) / 10**9

    def store_position_callback(self, msg : Odometry, drone_id : int):
        # pos_point = msg.pose.pose.position
        # self.drone_pos[drone_id] = (pos_point.x, pos_point.y, pos_point.z)
        self.drone_pos[drone_id] = msg.pose.pose.position

def calculate_target_priority(point1 : Point, point2 : Point, aoi2 : float, aoi_threshold2 : float, aoi_weight : float, violation_weight : float, alpha=1.0, beta=1.0, eps=0.0000001) -> float:
    """
    Computes the inverse priority of reaching point2 from point1, depending on the current scenario of the simulation.
    
    Takes:
    	point1, point2: ROS points
    	aoi_threshold2: the threshold of the AoI for point2
    	aoi2:           the CURRENT AoI of point2
    	aoi_weight:     the weight of the AoI in the final score
    	violation_weight: the weight of the violation in the final score
    	alpha, beta: scaling parametrs
    """
    p1 = np.array((point1.x, point1.y, point1.z))
    p2 = np.array((point2.x, point2.y, point2.z))
    
    euclidean = np.linalg.norm(p1 - p2)
    # if violation_weight > aoi_weight:
    if True:
        # POLICY 1
        """
        if (aoi_threshold2 - aoi2) < 0: # constraint violated
            aoi_bonus = (aoi2/aoi_threshold2) * abs(aoi_threshold2 - aoi2)
        elif (aoi_threshold2 - aoi2) == 0: # we don't want to deal with 0
            aoi_bonus = 1.0
        else: # legal
            aoi_bonus = (aoi2/aoi_threshold2) * (1 / (aoi_threshold2 - aoi2))
        """
        # POLICY 2
        aoi_bonus = aoi2/aoi_threshold2
        
    else:
        aoi_bonus = aoi2
    
    # result = (aoi_bonus*beta + eps) / (euclidean*alpha)
    result = -(aoi_bonus*beta + eps) / (np.exp(euclidean  * alpha))

    print("[MESSAGE]: priority of point:", point2, ": ", result, euclidean)
    return result

def main():

    time.sleep(3.0)
    
    rclpy.init()

    task_assigner = TaskAssigner()
    executor = MultiThreadedExecutor()
    executor.add_node(task_assigner)

    task_assigner.get_task_and_subscribe_to_drones()
    task_assigner.keep_patrolling()

    executor.spin()

    executor.shutdown()
    task_assigner.destroy_node()

    rclpy.shutdown()

