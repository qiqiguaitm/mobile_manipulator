#!/usr/bin/env python3

import rospy
import networkx as nx
from collections import deque

class DAGExecutor:
    def __init__(self):
        rospy.init_node('dag_executor')
        
        # Create a directed graph for the DAG
        self.graph = nx.DiGraph()
        
        # Define the execution order based on the design document
        self.build_dag()
        
        rospy.loginfo("DAG Executor initialized")

    def build_dag(self):
        """Build the DAG based on module dependencies from design.md"""
        # Add nodes (modules)
        modules = [
            'sensor_calibration',
            'inference_abstraction',
            'slam',
            'perception',
            'object_tracker',
            'target_filter',
            'task_router',
            'task_mgr',
            'chassis_planner',
            'arm_planner',
            'chassis_controller',
            'arm_controller'
        ]
        
        for module in modules:
            self.graph.add_node(module)
        
        # Add edges (dependencies)
        # Sensor Calibration provides data to SLAM, Perception, and ObjectTracker
        self.graph.add_edge('sensor_calibration', 'slam')
        self.graph.add_edge('sensor_calibration', 'perception')
        self.graph.add_edge('sensor_calibration', 'object_tracker')
        
        # Perception provides data to ObjectTracker
        self.graph.add_edge('perception', 'object_tracker')
        
        # ObjectTracker provides data to TargetFilter
        self.graph.add_edge('object_tracker', 'target_filter')
        
        # TargetFilter provides data to TaskRouter
        self.graph.add_edge('target_filter', 'task_router')
        
        # TaskRouter provides data to TaskManager
        self.graph.add_edge('task_router', 'task_mgr')
        
        # SLAM provides data to TaskManager
        self.graph.add_edge('slam', 'task_mgr')
        
        # TaskManager provides data to ChassisPlanner and ArmPlanner
        self.graph.add_edge('task_mgr', 'chassis_planner')
        self.graph.add_edge('task_mgr', 'arm_planner')
        
        # ChassisPlanner provides data to ChassisController
        self.graph.add_edge('chassis_planner', 'chassis_controller')
        
        # ArmPlanner provides data to ArmController
        self.graph.add_edge('arm_planner', 'arm_controller')
        
        # InferenceAbstraction provides data to Perception and ObjectTracker
        self.graph.add_edge('inference_abstraction', 'perception')
        self.graph.add_edge('inference_abstraction', 'object_tracker')

    def get_execution_order(self):
        """Get the topological sort order for DAG execution"""
        try:
            return list(nx.topological_sort(self.graph))
        except nx.NetworkXError:
            rospy.logerr("Cycle detected in DAG!")
            return []

    def execute_dag(self):
        """Execute the DAG in topological order"""
        execution_order = self.get_execution_order()
        
        if not execution_order:
            rospy.logerr("Cannot execute DAG due to cycle or other error")
            return
        
        rospy.loginfo("Execution order: %s", " -> ".join(execution_order))
        
        # In a real implementation, this would actually execute each module
        # For now, we'll just simulate the execution
        for module in execution_order:
            rospy.loginfo("Executing module: %s", module)
            # Simulate some work
            rospy.sleep(0.1)
        
        rospy.loginfo("DAG execution completed")

    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Execute the DAG periodically
            self.execute_dag()
            rate.sleep()

if __name__ == '__main__':
    try:
        executor = DAGExecutor()
        executor.run()
    except rospy.ROSInterruptException:
        pass