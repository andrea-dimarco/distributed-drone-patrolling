import time
import random
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

import dearpygui.dearpygui as dpg

from rosgraph_msgs.msg import Clock
from iot_project_interfaces.srv import TaskAssignment

TIME_PER_TARGET = 30
MAX_TIME = 120

# ------------------------------ IoT Project Grader Display -------------------------------
# This Node calculates the current score of the simulation, by listening to the simulation
# time and doing the appropriate calculations.
# Current implementation currently fixed for only 5 targets. Can be easily parametrized to
# work with multiple targets.
# -----------------------------------------------------------------------------------------


class GradeDisplay(Node):

    def __init__(self):
        super().__init__('grade_display')

        self.simulation_name = ""
        self.simulation_time = MAX_TIME

        self.aoi_weight = 1.0
        self.violation_weight = 1.0
        self.fairness_weight = 1.0

        # parameters for tracking the targets' values
        self.target_tags = []
        self.aoi_tags = []

        self.aoi_x_values = []
        self.aoi_y_values = []
        
        self.last_visits = []
        self.expiration_times = []
        self.timer_tag = "timer"
        self.metrics_tag = "metrics"
        self.score_tag = "score"
        self.display_thread = None

        self.clock = 0

        # subscribe to the clock of the gazebo world
        self.time_subscriber = self.create_subscription(
            Clock,
            '/world/iot_project_world/clock',
            self.store_time_callback,
            10
        )

        # create a client for the target service
        self.task_client = self.create_client(
            TaskAssignment,
            '/task_assigner/get_task'
        )


        self.get_logger().info("Grader Display is starting.")
        self.get_logger().info("Waiting for targets' data Service...")

        while not self.task_client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info("Service online. Display will now run.")

        self.create_timer(0.1, self.update_times)
        self.start_gui()
        

    def update_times(self):
        future = self.task_client.call_async(TaskAssignment.Request())
        future.add_done_callback(self.update_times_callback)
    
    def update_times_callback(self, res : Future):
        
        result : TaskAssignment.Response = res.result()
        self.simulation_name = result.simulation_name
        self.simulation_time = result.simulation_time
        no_targets = len(result.target_positions)
        self.last_visits = result.last_visits
        self.expiration_times = result.target_thresholds

        self.aoi_weight = result.aoi_weight
        self.violation_weight = result.violation_weight
        self.fairness_weight = result.fairness_weight


        if len(self.target_tags) == 0:
            self.target_tags = ["target_%d" % i for i in range(no_targets)]
            self.aoi_tags = ["aoi_%d" % i for i in range(no_targets)]

            self.aoi_x_values = [[] for i in range(no_targets)]
            self.aoi_y_values = [[] for i in range(no_targets)]

    def store_time_callback(self, msg : Clock):
        self.clock = msg.clock.sec * 10**9 + msg.clock.nanosec

    def start_gui_function(self):

        while len(self.target_tags) == 0:
            time.sleep(0.5)

        test = None

        ENTRY_WIDTH = 150
        WINDOW_WIDTH = (ENTRY_WIDTH + 10)*len(self.target_tags) - 5
        WINDOW_PADDING = 17
        WINDOW_WIDTH_INNER = WINDOW_WIDTH - WINDOW_PADDING
        WINDOW_HEIGHT = 616


        dpg.create_context()
        dpg.create_viewport(title='IoT Project Grader', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

        with dpg.window(tag = "Test", pos=(0,0), width=WINDOW_WIDTH, height=WINDOW_HEIGHT, no_title_bar=True, no_resize=True, no_move=True, no_scrollbar=True):

            dpg.add_button(label = "Simulation name", width = WINDOW_WIDTH_INNER, height = 30, tag="sim_name")
            dpg.add_button(label = "Time left to visit:", width = WINDOW_WIDTH_INNER )

            
            with dpg.group(horizontal=True):
                for i in range(len(self.target_tags)):
                    with dpg.group():
                        dpg.add_button(label="Target %d" % (i+1), width=ENTRY_WIDTH)
                        dpg.add_slider_float(tag=self.target_tags[i], max_value = self.expiration_times[i], width = ENTRY_WIDTH, height = 150, vertical = True)

            dpg.add_button(label = "Normalized AoI Values:", width = WINDOW_WIDTH_INNER )
            with dpg.group(horizontal=True):
                for i in range(len(self.target_tags)):
                        with dpg.plot(width=ENTRY_WIDTH, height=100):
                            with dpg.plot_axis(dpg.mvXAxis, label="Time", no_tick_labels=True):
                                dpg.set_axis_limits(dpg.last_item(), 0, self.simulation_time)
                            with dpg.plot_axis(dpg.mvYAxis):
                                dpg.set_axis_limits(dpg.last_item(), 0, 1)
                                dpg.set_axis_ticks(dpg.last_item(), (("0",0), ("", 0.5), ("1",1)))
                                dpg.add_line_series([0], [0], tag=self.aoi_tags[i])
                                dpg.add_line_series([0, self.simulation_time], [self.expiration_times[i]/self.simulation_time]*2)
                              


            dpg.add_button(label = "Weights", width = WINDOW_WIDTH_INNER, tag="weights")                            
                    
            dpg.add_button(tag=self.timer_tag, label = "TIMER", width = WINDOW_WIDTH_INNER, height = 60)
            dpg.add_button(tag=self.metrics_tag, label = "METRICS", width = WINDOW_WIDTH_INNER, height = 60)
            dpg.add_button(tag=self.score_tag, label = "SCORE", width = WINDOW_WIDTH_INNER, height = 90)


        dpg.setup_dearpygui()
        dpg.show_viewport()
        clock_float = 0.0
        aoi_score = 0.0
        violation_malus = 0.0
        total_score = 0.0

        while dpg.is_dearpygui_running():


            step_size = 0
            if clock_float < self.simulation_time:
                step_size = self.clock / 10** 9 - clock_float
            clock_float = self.clock / 10**9

            time_left = max(0, self.simulation_time - clock_float)

            dpg.configure_item(self.timer_tag, label="TIME LEFT: %d" % time_left)
            dpg.configure_item("sim_name", label=self.simulation_name)
            dpg.configure_item("weights", label="Weights - AoI: %.2f   Violation: %.2f   Fairness: %.2f" % (self.aoi_weight, self.violation_weight, self.fairness_weight))
                

            evaluated_fariness = 1

            for t in range(len(self.last_visits)):

                if time_left == 0:
                    break

                target_time_float = self.last_visits[t] / 10**9

                target_time_left = max(0, self.expiration_times[t] - (clock_float - target_time_float))

                aoi_step = ((clock_float - target_time_float) / self.simulation_time)
                aoi_score += aoi_step * step_size
                
                if target_time_left <= 0:
                    violation_step = (clock_float - target_time_float - self.expiration_times[t]) / self.simulation_time
                    violation_malus += violation_step * step_size

                aoi_value_x = clock_float
                aoi_value_y = (clock_float - target_time_float) / self.simulation_time

                normalized_threshold = self.expiration_times[t] / self.simulation_time


                
                total_score_aoi_contribution = max(0, (1 - aoi_step) - (1 - normalized_threshold))
                total_score_violation_contribution = (1 - aoi_step) - total_score_aoi_contribution
                

                total_score += (total_score_aoi_contribution * self.aoi_weight + total_score_violation_contribution * self.violation_weight) * step_size

                self.aoi_x_values[t].append(aoi_value_x)
                self.aoi_y_values[t].append(aoi_value_y)
                

                dpg.set_value(self.target_tags[t], value=target_time_left)
                dpg.configure_item(self.target_tags[t], max_value=self.expiration_times[t])
                dpg.configure_item(self.aoi_tags[t], x=self.aoi_x_values[t], y=self.aoi_y_values[t])
                


            aoi_values = []

            for a in range(len(self.aoi_tags)):
                aoi_values.append(max(self.aoi_y_values[a]))
        

            #print(max(aoi_values), min(aoi_values))
            evaluated_fariness = 1 - ((max(aoi_values) - min(aoi_values)))

            

            dpg.configure_item(self.metrics_tag, label="Cumulative AoI: \t\t\t %.2f\nCumulative Violation:\t\t%.2f\nEvaluated Fairness: \t\t %.2f" % (aoi_score, violation_malus, evaluated_fariness))

            score_with_fairness = total_score * (1 + evaluated_fariness * self.fairness_weight)

            dpg.configure_item(self.score_tag, label="SCORE: %.2f" % score_with_fairness)
         

            dpg.render_dearpygui_frame()
            time.sleep(0.05)


        dpg.destroy_context()
        dpg.stop_dearpygui()



    def start_gui(self):
        self.display_thread = Thread(target=self.start_gui_function)
        self.display_thread.start()


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    grade_display = GradeDisplay()

    executor.add_node(grade_display)
    executor.spin()

    executor.shutdown()
    grade_display.destroy_node()
    rclpy.shutdown()