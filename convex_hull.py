from typing import List
import time
import math

from which_pyqt import PYQT_VER

if PYQT_VER == 'PYQT5':
    from PyQt5.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT4':
    from PyQt4.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT6':
    from PyQt6.QtCore import QLineF, QPointF, QObject
else:
    raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

# Some global color constants that might be useful
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Global variable that controls the speed of the recursion automation, in seconds
#
PAUSE = 0.25


#
# This is the class you have to complete.
#


class ConvexHullSolver(QObject):

    # Class constructor
    def __init__(self):
        super().__init__()
        self.pause = False

    # Some helper methods that make calls to the GUI, allowing us to send updates
    # to be displayed.

    def showTangent(self, line, color):
        self.view.addLines(line, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseTangent(self, line):
        self.view.clearLines(line)

    def blinkTangent(self, line, color):
        self.showTangent(line, color)
        self.eraseTangent(line)

    def showHull(self, polygon, color):
        self.view.addLines(polygon, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseHull(self, polygon):
        self.view.clearLines(polygon)

    def showText(self, text):
        self.view.displayStatusText(text)

    # This is the method that gets called by the GUI and actually executes
    # the finding of the hull
    def compute_hull(self, points, pause, view):
        self.pause = pause
        self.view = view
        assert (type(points) == list and type(points[0]) == QPointF)

        t1 = time.time()
        # DONE: SORT THE POINTS BY INCREASING X-VALUE
        # print("Points before: ", points)
        points = sorted(points, key=lambda x: x.x())
        # print("Points after: ", points)

        t2 = time.time()
        t3 = time.time()
        # this is a dummy polygon of the first 3 unsorted points
        # polygon = [QLineF(points[i], points[(i + 1) % 3]) for i in range(3)]
        polygon = self.convex_solver_divide_and_conquer(points, pause, view)
        # print(points) - DEBUGGING STATEMENT
        t4 = time.time()
        # when passing lines to the display, pass a list of QLineF objects.  Each QLineF
        # object can be created with two QPointF objects corresponding to the endpoints
        hull = [QLineF(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon))]
        self.showHull(hull, BLUE)
        self.showText('Time Elapsed (Convex Hull): {:3.3f} sec'.format(t4 - t3))

    # This functions divides all the points to find the left hull and right hull of the provided points recursively
    # and then combines both of them by finding the upper and lower tangent between both and combining them.
    # Time Complexity: O(nlog(n)), Space Complexity at its worse: 0(log(n))
    # Explanation: All the subsections in this algorithm, especially the combination of both hulls take O(n) time.
    # In addition, we are dividing the provided points by two each time (n/2, n/4, n/6...infinity)
    # so complexity is nlog(n).
    def convex_solver_divide_and_conquer(self, points, pause, view):
        num_points = len(points)

        if num_points == 1:
            return points

        # Start from index 0 to half
        left_hull = self.convex_solver_divide_and_conquer(points[:num_points // 2], pause, view)
        # Start from half to end
        right_hull = self.convex_solver_divide_and_conquer(points[num_points // 2:], pause, view)

        # Initialize a combined hull to return as an empty variable of type list to be able to append to it
        combined_hull = []

        # Check if there are less than 2 points on each hull. If that's the case, then combine them and append to
        # our combined hull
        # Time Complexity: O(n)
        if len(left_hull) == 1 and len(right_hull) == 1:
            left_hull.extend(right_hull)
            combined_hull.extend(left_hull)
            return combined_hull

        # Section: Spider crawl approach :spider: #

        # Step 1: Find left-most and right-most variables to start with our spider crawl approach
        # Description: Find the left-most point of the right hull, and then find the right-most point of the left hull
        # This will allow to have starting points to calculate slopes and scale up or down depending on the desired
        # tangent.
        # Time Complexity: O(n)

        left_most_right_index = self.find_left_most_index(right_hull)
        right_most_left_index = self.find_right_most_index(left_hull)

        # Step 2: Find lower and upper tangent
        # Time Complexity: O(n)

        # 2.1 - Upper Tangent
        left_index = right_most_left_index  # i
        right_index = left_most_right_index  # j

        left_hull_upper_point_found = True
        right_hull_upper_point_found = True

        slope = self.find_slope(left_hull, left_index, right_hull, right_index)

        while right_hull_upper_point_found or left_hull_upper_point_found:
            left_hull_upper_point_found = False
            right_hull_upper_point_found = False
            while True:
                new_slope = self.find_new_slope1(left_hull, left_index - 1, right_hull, right_index)
                if new_slope < slope:
                    left_hull_upper_point_found = True
                    slope = new_slope
                    left_index = (left_index - 1) % len(left_hull)
                else:
                    break
            while True:
                new_slope = self.find_new_slope2(left_hull, left_index, right_hull, right_index + 1)
                if new_slope > slope:
                    right_hull_upper_point_found = True
                    slope = new_slope
                    right_index = (right_index + 1) % len(right_hull)
                else:
                    break

        upper_tangent = (left_index, right_index)

        # 2.2 - Lower Tangent
        left_index = right_most_left_index  # i
        right_index = left_most_right_index  # j

        left_hull_lower_point_found = True
        right_hull_lower_point_found = True

        slope = self.find_slope(left_hull, left_index, right_hull, right_index)

        while right_hull_lower_point_found or left_hull_lower_point_found:
            left_hull_lower_point_found = False
            right_hull_lower_point_found = False
            while True:
                new_slope = self.find_new_slope1(left_hull, left_index + 1, right_hull, right_index)
                if new_slope > slope:
                    left_hull_lower_point_found = True
                    slope = new_slope
                    left_index = (left_index + 1) % len(left_hull)
                else:
                    break
            while True:
                new_slope = self.find_new_slope2(left_hull, left_index, right_hull, right_index - 1)
                if new_slope < slope:
                    right_hull_lower_point_found = True
                    slope = new_slope
                    right_index = (right_index - 1) % len(right_hull)
                else:
                    break

        lower_tangent = (left_index, right_index)

        self.show_recursion(left_hull, right_hull, upper_tangent, lower_tangent)

        # Step 3:
        # Combine the two hulls
        # Time Complexity: O(n)
        low = lower_tangent[0]
        combined_hull.append(left_hull[low])

        while low != upper_tangent[0]:
            low = (low + 1) % len(left_hull)
            combined_hull.append(left_hull[low])

        up = upper_tangent[1]
        combined_hull.append(right_hull[up])

        while up != lower_tangent[1]:
            up = (up + 1) % len(right_hull)
            combined_hull.append(right_hull[up])

        return combined_hull

    def find_left_most_index(self, hull: List[QPointF]) -> int:
        return hull.index(min(hull, key=lambda x: x.x()))

    def find_right_most_index(self, hull: List[QPointF]) -> int:
        return hull.index(max(hull, key=lambda x: x.x()))

    def find_slope(self, left_hull: List[QPointF], i, right_hull: List[QPointF], j) -> int:
        return (right_hull[j].y() - left_hull[i].y()) / (right_hull[j].x() - left_hull[i].x())

    def find_new_slope1(self, left_hull: List[QPointF], i, right_hull: List[QPointF], j) -> int:
        return (right_hull[j].y() - left_hull[i % len(left_hull)].y()) / \
               (right_hull[j].x() - left_hull[i % len(left_hull)].x())

    def find_new_slope2(self, left_hull: List[QPointF], i, right_hull: List[QPointF], j) -> int:
        return (right_hull[j % len(right_hull)].y() - left_hull[i].y()) / \
               (right_hull[j % len(right_hull)].x() - left_hull[i].x())

    def show_recursion(self, left_hull, right_hull, up_tangent, low_tangent):
        left = [QLineF(left_hull[i], left_hull[(i + 1) % len(left_hull)]) for i in range(len(left_hull))]
        right = [QLineF(right_hull[i], right_hull[(i + 1) % len(right_hull)]) for i in range(len(right_hull))]
        up = QLineF(left_hull[up_tangent[0]], right_hull[up_tangent[1]])
        low = QLineF(left_hull[low_tangent[0]], right_hull[low_tangent[1]])
        self.showHull(left, GREEN)
        self.showHull(right, GREEN)
        self.showTangent([up, low], RED)
        self.eraseHull(left_hull)
        self.eraseHull(right_hull)
        self.eraseTangent([up, low])

    # """ Failed Attempt
    # def convex_solver_divide_conquer(self, points):  # Divide and conquer algorithm
    #     num_points = len(points)
    #
    #     if num_points < 2:
    #         return self.clockwise_order(points)
    #     else:
    #         left_hull = self.convex_solver_divide_conquer(self, points[:num_points // 2])
    #         right_hull = self.convex_solver_divide_conquer(self, points[num_points // 2:])
    #         return combine(left_hull, right_hull)
    #
    # def clockwise_order(self, points: List[QPointF]) -> List[QPointF]:
    #     left_most = points[0]  # Leftmost will always be the index 0 in the points
    #
    #     if len(points) == 2:
    #         return [left_most, points[1]]
    #
    #     assert (len(points) == 3)  # Check and compare the slopes
    #
    #     slope2_point = self.calculate_slope(self, left_most, points[1])
    #     slope3_point = self.calculate_slope(self, left_most, points[2])
    #
    #     if slope2_point < slope3_point:  # Put the calculated slopes in order
    #         return [left_most, points[2], points[1]]
    #     return [left_most, points[1], points[2]]
    #
    # def calculate_slope(self, left_most: QPointF, next_point: QPointF):
    #     return (next_point.y() - left_most.y()) / (next_point.x() - left_most.x())  # Rise (y) / Run (x)
    #
    # def combine(self, left_hull: List[QPointF], right_hull: List[QPointF]) -> List[QPointF]:
    #     # Variables to "restart" indexes for each hull
    #
    #     index_left_most_right = 0  # Should always be zero
    #     index_right_most_left = self.find_right_most(left_hull)
    #
    #     # Spider crawl approach #
    #
    #     # Variables to find top and bottoms
    #     current_left_index = index_right_most_left
    #     current_right_index = index_left_most_right
    #
    #     # Find top of left hull
    #     current_slope = self.calculate_slope(left_hull[current_left_index], right_hull[current_right_index])
    #     new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #
    #     while new_slope < current_slope:
    #         current_slope = new_slope
    #         current_left_index -= 1
    #         new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #
    #     # Find top of right hull
    #     new_slope = self.calculate_slope(left_hull[current_left_index], right_hull[current_right_index + 1])
    #     while new_slope > current_slope:
    #         current_slope = new_slope
    #         current_right_index += 1
    #         new_slope = self.calculate_slope(left_hull[current_left_index], right_hull[current_right_index + 1])
    #
    #     # "Restart" indexes to find bottoms
    #     current_left_index = index_right_most_left
    #     current_right_index = index_left_most_right
    #
    #     # Find bottom of left hull
    #     current_slope = self.calculate_slope(left_hull[current_left_index], right_hull[current_right_index])
    #     new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #     while new_slope > current_slope:
    #         current_slope = new_slope
    #         current_left_index += 1
    #         new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #
    #     # Find bottom of right hull
    #     new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #     while new_slope > current_slope:
    #         current_slope = new_slope
    #         current_left_index += 1
    #         new_slope = self.calculate_slope(left_hull[current_left_index - 1], right_hull[current_right_index])
    #
    #     return
    #
    #
    # def find_right_most(self, points: List[QPointF]) -> int:
    #     for index in range(len(points)):
    #         if points[index].x() < points[index + 1].x() or index == len(points) - 1:
    #             return index
    #
    #
    #     """
