#!/usr/bin/env python3

import copy
from math import pi, cos, sin
from MoveGroupPythonInterface import MoveGroupPythonInterface
from ServoDriver import ServoDriver
from vector3d.vector import Vector

class CorradoTrajectoryController(object):
    """CorradoTrajectoryController"""

    def __init__(self):
        super(CorradoTrajectoryController, self).__init__()

        self.corrado_move_group = MoveGroupPythonInterface()
        self.driver_controller = ServoDriver(n_joints=6, servo_freq=50)

        self.CELL_WIDTH = 0.065
        self.cell_center3d = Vector(0.0, 0.21, 0.065)

        self.home_position = [0.15, 0.15, 0.125]
        self.cell_centers3d = list()
        self.cell_centers_init()
        self.up_trasl3d = Vector(0.0, 0.0, 0.05)
        self.radius = 0.02

    def add_waypoint(self, waypoints_list, points3d):
        wpose = self.corrado_move_group.move_group.get_current_pose().pose
        wpose.position.x = 0.0
        wpose.position.y = 0.0
        wpose.position.z = 0.0
        for point3d in points3d:
            wpose.position.x += point3d.x
            wpose.position.y += point3d.y
            wpose.position.z += point3d.z
        waypoints_list.append(copy.deepcopy(wpose))

    @staticmethod
    def extract_traj_from_plan(plan):
        trajectory = list()
        for point in plan.joint_trajectory.points:
            trajectory.append(point.positions)
        return trajectory

    def execute_plan(self, plan):
        traj = self.extract_traj_from_plan(plan)
        self.driver_controller.execute_trajectory(traj)
       

    def cell_centers_init(self):
        self.cell_centers3d.append(self.cell_center3d + Vector(-self.CELL_WIDTH, self.CELL_WIDTH, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(0.0, self.CELL_WIDTH, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(self.CELL_WIDTH, self.CELL_WIDTH, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(-self.CELL_WIDTH, 0.0, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(0.0, 0.0, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(self.CELL_WIDTH, 0.0, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(-self.CELL_WIDTH, -self.CELL_WIDTH, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(0.0, -self.CELL_WIDTH, 0.0))
        self.cell_centers3d.append(self.cell_center3d + Vector(self.CELL_WIDTH, -self.CELL_WIDTH, 0.0))

    def homing(self):
        self.corrado_move_group.go_to_pose_goal(self.home_position)


    def draw_point(self, cell_index):
        point3d = self.cell_centers3d[cell_index]
        waypoints = list()

        wpose = self.corrado_move_group.move_group.get_current_pose().pose
        start_position = Vector(wpose.position.x, wpose.position.y, wpose.position.z)

        self.add_waypoint(waypoints, [start_position])
        self.add_waypoint(waypoints, [point3d, self.up_trasl3d])
        self.add_waypoint(waypoints, [point3d])
        self.add_waypoint(waypoints, [point3d, self.up_trasl3d])
        self.add_waypoint(waypoints, [start_position])

        plan, _ = self.corrado_move_group.plan_cartesian_path(waypoints)
        self.corrado_move_group.display_trajectory(plan)
        #self.corrado_move_group.execute_plan(plan)
        self.execute_plan(plan)
        

    def draw_x(self, cell_index):
        center3d = self.cell_centers3d[cell_index]
        waypoints = list()

        wpose = self.corrado_move_group.move_group.get_current_pose().pose
        start_position = Vector(wpose.position.x, wpose.position.y, wpose.position.z)

        alto_sx = center3d + Vector(-self.radius, self.radius, 0.0) 
        alto_dx = center3d + Vector(self.radius, self.radius, 0.0)
        basso_sx = center3d + Vector(-self.radius, -self.radius, 0.0)
        basso_dx = center3d + Vector(self.radius, -self.radius, 0.0)

        self.add_waypoint(waypoints, [start_position])

        self.add_waypoint(waypoints, [alto_sx, self.up_trasl3d])
        self.add_waypoint(waypoints, [alto_sx])
        self.add_waypoint(waypoints, [basso_dx])
        self.add_waypoint(waypoints, [basso_dx, self.up_trasl3d])
        
        self.add_waypoint(waypoints, [alto_dx, self.up_trasl3d])
        self.add_waypoint(waypoints, [alto_dx])
        self.add_waypoint(waypoints, [basso_sx])
        self.add_waypoint(waypoints, [basso_sx, self.up_trasl3d])

        self.add_waypoint(waypoints, [start_position])

        plan, _ = self.corrado_move_group.plan_cartesian_path(waypoints)
        self.corrado_move_group.display_trajectory(plan)
        #self.corrado_move_group.execute_plan(plan)
        self.execute_plan(plan)
    
    def draw_circle(self, cell_index):
        center3d = self.cell_centers3d[cell_index]
        waypoints = list()

        wpose = self.corrado_move_group.move_group.get_current_pose().pose
        start_position = Vector(wpose.position.x, wpose.position.y, wpose.position.z)
        self.add_waypoint(waypoints, [start_position])

        number_waypoints = 25
        for i in range(number_waypoints+1):
            waypoint_x = center3d.x + self.radius*cos(i/number_waypoints*2*pi)
            waypoint_y = center3d.y + self.radius*sin(i/number_waypoints*2*pi)
            self.add_waypoint(waypoints, [Vector(waypoint_x, waypoint_y, center3d.z)])

        self.add_waypoint(waypoints, [start_position])

        plan, _ = self.corrado_move_group.plan_cartesian_path(waypoints)
        self.corrado_move_group.display_trajectory(plan)
        #self.corrado_move_group.execute_plan(plan)
        self.execute_plan(plan)

