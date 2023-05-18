import time
import random
from sklearn.cluster import KMeans
from threading import Thread
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from iot_project_interfaces.srv import TaskAssignment
from iot_project_solution_interfaces.action import PatrollingAction

class TaskAssigner(Node):

    def __init__(self):

        super().__init__('task_assigner')
            
        self.task = None
        self.no_drones = 0
        self.targets = []
        self.thresholds = []
        self.violation_w = 0
        self.fairness_w = 0
        self.aoi_w = 0
        self.target_clusters = []
        self.action_servers = []
        self.current_tasks =  []
        self.idle = []

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

        # get all weights for final score
        self.violation_w = task.violation_weight
        self.fairness_w = task.fairness_weight
        self.aoi_w = task.aoi_weight

        #maybe have a variable that decides which solution to use based on number of targets and drones
        #self.difficulty = 0

        #here we compute the clusters for each drone
        #simple array splitting to test the methodology
        #self.target_clusters = np.array_split(np.array(task.target_positions),task.no_drones)

        print('CREATING TMP ARRAY OF SAMPLES')
        tmp_array=np.array([(a.x,a.y,a.z) for a in task.target_positions])

        print('CLUSTERING SAMPLES\n')
        kmeans= KMeans(n_clusters=task.no_drones,random_state=0, n_init='auto').fit(tmp_array)
        
        #print("SORTING CLUSTERS\n")
        #need to assign drone based on distance to cluster and sort each cluster to get optimal order of visit
        
        tmp_cluster = [tmp_array[kmeans.labels_ == a] for a in range(self.no_drones)]

        # converting to Point matrix
        self.target_clusters = [[Point(x=el[0],y=el[1],z=el[2]) for el in cluster] for cluster in tmp_cluster]

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
    def submit_task(self, drone_id, targets_to_patrol = None):

        self.get_logger().info("Submitting task for drone X3_%s" % drone_id)
    
        while not self.action_servers[drone_id].wait_for_server(timeout_sec = 1.0):
            return

        self.idle[drone_id] = False

        #assign target to drone
        if not targets_to_patrol:
            #targets_to_patrol = self.targets.copy()
            #random.shuffle(targets_to_patrol)
            targets_to_patrol = self.target_clusters[drone_id]
            print(targets_to_patrol)

        patrol_task =  PatrollingAction.Goal()
        patrol_task.targets = targets_to_patrol

        patrol_future = self.action_servers[drone_id].send_goal_async(patrol_task)

        # This is a new construct for you. Basically, callbacks have no way of receiving arguments except
        # for the future itself. We circumvent such problem by creating an inline lambda functon which stores
        # the additional arguments ad-hoc and then calls the actual callback function
        patrol_future.add_done_callback(lambda future, d = drone_id : self.patrol_submitted_callback(future, d))


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

