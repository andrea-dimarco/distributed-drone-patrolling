import time
import random

from threading import Thread
import numpy as np
from sklearn.cluster import SpectralClustering, KMeans
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from iot_project_interfaces.srv import TaskAssignment
from iot_project_solution_interfaces.action import PatrollingAction
from iot_project_interfaces.msg import TargetsTimeLeft

from .ant_colony import find_patrol_route

class TaskAssigner(Node):

    def __init__(self):

        super().__init__('task_assigner')
        self.task = None
        self.no_drones = 0
        self.drone_pos = [] # we save the position of the drone after each task (the last point visited)
        self.targets = []
        self.thresholds = []
        self.thresholds_dict = {}
        self.cluster_map = []
        self.action_servers = []
        self.current_tasks =  []
        self.idle = []
        self.last_visits = []
        self.targets_time_left = []

        self.sim_time = 0


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

        self.targets_time_left_topic = self.create_subscription(
            TargetsTimeLeft,
            '/task_assigner/targets_time_left',
            self.store_targets_time_left_callback,
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
        for i in range(self.no_drones):
            self.drone_pos.append(Point(x=0.0, y=0.0, z=0.0))

        self.targets = task.target_positions
        self.thresholds = task.target_thresholds
        #can't directly use Point object
        self.thresholds_dict = {tuple((k.x,k.y,k.z)):v for k,v in zip(self.targets, self.thresholds)}

        print("[MESSAGE] Printing Threshold Dict",self.thresholds_dict)
        self.current_tasks = [None]*self.no_drones
        self.idle = [True] * self.no_drones
        self.last_visits = task.last_visits

        # get all weights for final score
        self.violation_w = task.violation_weight
        self.fairness_w = task.fairness_weight
        self.aoi_w = task.aoi_weight

        ### dictionary with paths to assign to drones
        self.patrol_routes = { drone_id : [] for drone_id in range(self.no_drones)}

        # here we compute the clusters for each drone
        tmp_array = np.array([(a.x,a.y,a.z) for a in task.target_positions])
        clustering_method = SpectralClustering(n_clusters=task.no_drones,random_state=0, n_init='auto').fit(tmp_array)
        
        #need to assign drone based on distance to cluster and sort each cluster to get optimal order of visit
        
        tmp_cluster = [tmp_array[clustering_method.labels_ == a] for a in range(self.no_drones)]

        # converting to Point matrix
        self.target_clusters = [[Point(x=el[0],y=el[1],z=el[2]) for el in cluster] for cluster in tmp_cluster]

        self.cluster_map = [[self.targets.index(p) for p in cluster] for cluster in self.target_clusters]
        
        print("[MESSAGE] Printing Cluster Map",self.cluster_map)

        # Now create a client for the action server of each drone
        for d in range(self.no_drones):
            self.action_servers.append(
                ActionClient(
                    self,
                    PatrollingAction,
                    'X3_%d/patrol_targets' % d,
                )
            )
    
    # This method starts on a separate thread an ever-going patrolling task, it does that
    # by checking the idle state value of every drone and submitting a new goal as soon as
    # that value goes back to True
    def keep_patrolling(self):

        def keep_patrolling_inner():
            while True:
                for d in range(self.no_drones):
                    if self.idle[d]:

                        Thread(target=self.submit_task, args=(d,)).start()

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
        print("[MESSAGE] Action Server", self.action_servers)
        while not self.action_servers[drone_id].wait_for_server(timeout_sec = 1.0):
            return

        self.idle[drone_id] = False
        print("[MESSAGE] ASSIGNINING TARGETS")
        target_i = None 

        # start computing patrol routes in advance
        # because the computation might take time
        if not targets_to_patrol:


            # NEED TO CALL TASK ASSIGNER AND GET TASK UPDATE
            drone_cluster = self.target_clusters[drone_id]
            drone_pos = self.drone_pos[drone_id]

            # Here we decide which strategy to use
            # maybe change based on number of drones/targets?

            #targets_to_patrol = self.greedy_patrol(drone_cluster,drone_id,drone_pos)
            targets_to_patrol = self.ant_patrol(drone_cluster,drone_id,drone_pos)   
            
            # Compute target with maximum priority
            # update the drone's position
            self.drone_pos[drone_id] = targets_to_patrol[-1]

        patrol_task = PatrollingAction.Goal()
        #patrol_task.targets = targets_to_patrol
        patrol_task.targets = targets_to_patrol
        patrol_future = self.action_servers[drone_id].send_goal_async(patrol_task)

        # This is a new construct for you. Basically, callbacks have no way of receiving arguments except
        # for the future itself. We circumvent such problem by creating an inline lambda functon which stores
        # the additional arguments ad-hoc and then calls the actual callback function
        patrol_future.add_done_callback(lambda future, d = drone_id : self.patrol_submitted_callback(future, d))

    def greedy_patrol(self,drone_cluster,drone_id,drone_pos):
            min_dist = float("inf")
            for i in range(len(drone_cluster)):
                # get the global index of the point
                global_point_index = self.cluster_map[drone_id][i]
                target_time_left = round(float(self.targets_time_left[global_point_index] / 10**9),2)
                # compute the AoI of the point
                point_aoi = self.thresholds[global_point_index] - target_time_left
                # get the threshold for the point
                aoi_threshold = self.thresholds[global_point_index]
                dist = calculate_target_priority(drone_pos, self.targets[global_point_index], point_aoi, aoi_threshold, self.aoi_w, self.violation_w, alpha=2.0, beta=1.0)
                if dist < min_dist:
                    min_dist = dist
                    target_i = global_point_index
            return [self.targets[target_i]]
    
    def ant_patrol(self,drone_cluster,drone_id,drone_pos):
        aois= []
        thresholds = []
        for i in range(len(drone_cluster)):
            # get the global index of the point
            global_point_index = self.cluster_map[drone_id][i]
            target_time_left = round(float(self.targets_time_left[global_point_index] / 10**9),2)
            # compute the AoI of the point
            point_aoi = self.thresholds[global_point_index] - target_time_left
            # get the threshold for the point
            aoi_threshold = self.thresholds[global_point_index]
            # append points to the list in order 
            # this way they have corresponding indexes to the array of points drone_cluster
            aois.append(point_aoi)
            thresholds.append(aoi_threshold)
        # compute a near-optimal path for the drone
        # we do this in advance to save time
        path = find_patrol_route(drone_cluster, aois, thresholds, self.aoi_w, self.violation_w, drone_pos)
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

    # Callback used to store time left for every target
    def store_targets_time_left_callback(self, msg):
        self.targets_time_left = msg.times


def calculate_target_priority(point1 : Point, point2 : Point, aoi2 : float, aoi_threshold2 : float, aoi_weight : float, violation_weight : float, alpha=2.0, beta=1.0) -> float:
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
    #print("[MESSAGE] Euclidean norm: %s" % euclidean)
    # if violation_weight > aoi_weight:
    if True:
        if (aoi_threshold2 - aoi2) < 0: # constraint violated
            aoi_bonus = (aoi2/aoi_threshold2) * abs(aoi_threshold2 - aoi2)
        elif (aoi_threshold2 - aoi2) == 0: # we don't want to deal with 0
            aoi_bonus = 1.0
        else: # legal
            aoi_bonus = (aoi2/aoi_threshold2) * (1 / (aoi_threshold2 - aoi2))
    else:
        aoi_bonus = aoi2
    
    result = -(aoi_bonus*beta) / (euclidean*alpha)
    #print("[MESSAGE] Inverse priority:", result)
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
