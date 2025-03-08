#!/usr/bin/env python3

"""
    # {Sarvesh Rajkumar}
    # {student id}
    # {sarvesh@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False
    def transformmatrix(self,x,y,yaw,origin):
      cosq = cos(yaw)
      sinq = sin(yaw)
      tmatrix = np.array([
        [cosq,-sinq,0,x - origin.position.x],
        [sinq,cosq,0,y - origin.position.y],
        [0,0,1,0],
        [0,0,0,1]
      ])
      return tmatrix
    
    def updateminmax(self,grid_map,xmin,xmax,ymin,ymax,i,j):
      if self.is_in_bounds(grid_map, i, j):
        if i < xmin:
          xmin = i
        elif i > xmax:
          xmax = i
        if j < ymin:
          ymin = j
        elif j > ymax:
          ymax = j
      
      return xmin,xmax,ymin,ymax
    
    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """
        origin = grid_map.get_origin()
        resolution = grid_map.get_resolution()
        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        rmx = pose.pose.position.x 
        rmy = pose.pose.position.y 
        # print("yaw \n",origin)
        tmatrix = self.transformmatrix(rmx,rmy,robot_yaw,origin)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        Rmx = rmx - origin.position.x
        Rmy = rmy - origin.position.y
        rx = int(Rmx /resolution)
        ry = int(Rmy /resolution)
        # print("RObot coor ",rmx,rmy)
        # The map resolution [m/cell]
        
        
        qinc = scan.angle_increment
        qmin = scan.angle_min
        qmax = scan.angle_max
        ranges = scan.ranges
        rmin = scan.range_min
        rmax = scan.range_max
        xmin = rx
        xmax = rx
        ymin = ry
        ymax = ry
        qcur = qmin
        i = 0
        obp = []
        xt = 0
        while i < len(ranges):
          if ranges[i] > rmin and ranges[i] < rmax:
            lrx = ranges[i] * cos(qcur)
            lry = ranges[i] * sin(qcur)
            
            point = np.array([lrx,lry,0,1])
            tpoint = np.dot(tmatrix,point)
            if tpoint[0] > xt:
              xt = tpoint[0]
            
            obp.append(tpoint[:2])         
            
          i+=1  
          qcur = qcur + qinc
        i = 0        
        while i < len(obp):
          xm = (obp[i][0])
          ym = (obp[i][1])
          x = int(xm/resolution) 
          y = int(ym/resolution)
          
          
          freenodes = self.raytrace((rx,ry),(x,y))
          for l in range(0,len(freenodes)):
            self.add_to_map(grid_map,freenodes[l][0],freenodes[l][1],self.free_space)
          i+=1
        
        i = 0        
        while i < len(obp):
          xm = (obp[i][0])
          ym = (obp[i][1])
          x = int(xm/resolution) 
          y = int(ym/resolution)
          if x < xmin:
            xmin = x
          if x > xmax:
            xmax = x
          if y < ymin:
            ymin = y
          if y > ymax:
            ymax = y
          self.add_to_map(grid_map,x,y,self.occupied_space) 
          i+=1

        """
        Fill in your solution here
        For E
        """
        
        
        


        """
        For C only!
        Fill in the update correctly below.
        """ 
        update = OccupancyGridUpdate()
        update.x = xmin
        update.y = ymin
        update.width = xmax - xmin + 1
        update.height = ymax - ymin + 1
        update.data = []
        for ix in range(ymin,ymax+1):
          for jy in range(xmin,xmax+1):
            update.data.append(grid_map[jy,ix])
        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        # print("Inflate")
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        
        """
        Fill in your solution here
        """
        radius = self.radius
        width = grid_map.get_width()
        height = grid_map.get_height()
        coor = []
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
              if i**2 + j**2 <= radius**2:
                coor.append((i,j))
        
        for x in range(width):
          for y in range(height):
              if grid_map[x, y] == self.occupied_space:
                  for i in range(len(coor)):                    
                    cx = x + coor[i][0]
                    cy = y + coor[i][1]
                    if self.is_in_bounds(grid_map, cx, cy) and grid_map[cx,cy]!=self.occupied_space:
                        self.add_to_map(grid_map, cx, cy, self.c_space)


        
        # Return the inflated map
        return grid_map
