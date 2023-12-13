from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
	from PyQt5.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT4':
	from PyQt4.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT6':
	from PyQt6.QtCore import QLineF, QPointF, QObject
else:
	raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

import time

# Some global color constants that might be useful
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)

# Global variable that controls the speed of the recursion automation, in seconds
PAUSE = 0.25

#
# This is the class you have to complete.
#
LEFTMOST_PT_IDX = 0 # In hulls created, leftmost points will be index 0

#
# This is the class you have to complete.
#
class ConvexHullSolver(QObject):

    # Class constructor
    def __init__( self):
        super().__init__()
        self.pause = False
        
    # Some helper methods that make calls to the GUI, allowing us to send updates to be displayed.
    def showTangent(self, line, color):
        self.view.addLines(line,color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseTangent(self, line):
        self.view.clearLines(line)

    def blinkTangent(self,line,color):
        self.showTangent(line,color)
        self.eraseTangent(line)

    def showHull(self, polygon, color):
        self.view.addLines(polygon,color)
        if self.pause:
            time.sleep(PAUSE)
        
    def eraseHull(self,polygon):
        self.view.clearLines(polygon)
        
    def showText(self,text):
        self.view.displayStatusText(text)
    
    # This is the method that gets called by the GUI and actually executes the finding of the hull  
    def compute_hull( self, points, pause, view):
        self.pause = pause
        self.view = view
        assert( type(points) == list and type(points[0]) == QPointF )

        sortedPoints = sorted(points, key=lambda point: point.x())
        
        t3 = time.time()
        polygon = self.convex_hull(self.divide_and_conquer(sortedPoints))
        t4 = time.time()

        # when passing lines to the display, pass a list of QLineF objects.  Each QLineF
        # object can be created with two QPointF objects corresponding to the endpoints
        self.showHull(polygon, BLUE)
        self.showText('Time Elapsed (Convex Hull): {:3.3f} sec'.format(t4-t3))

    # This function takes in a polygon represented as a list of points and returns the convex hull of the polygon 
    # It does this by creating a line between each sequential pair of points in the polygon 
    # Time complexity is O(n) | Space complexity is O(n) 
    def convex_hull(self, polygon): 
        return [QLineF(polygon[i], polygon[(i+1) % len(polygon)]) for i in range(len(polygon))]

    # This function takes a set of points, recursively divides them in half, 
    # finds the convex hull of each half, and then merges the two hulls together 
    # to get the full convex hull using a divide and conquer approach 
    # Time complexity is O(nlogn) due to the divide and conquer approach | Space complexity is O(n) to store the output
    def divide_and_conquer(self, points):
        # Base case: If there is only 1 point, return the point  
        if len(points) == 1:
            return points  

        # DIVIDE the set of points in half
        leftPoints = points[:len(points)//2]  
        rightPoints = points[len(points)//2:]
        
        # CONQUER: Recursively find the convex hull on the left and right halves
        left_hull = self.divide_and_conquer(leftPoints)
        right_hull = self.divide_and_conquer(rightPoints)  

        # Merge the two hulls and return the result
        return self.merge(left_hull, right_hull)


    # This function merges two convex hulls
    # Time Complexity: O(n) | Space Complexity: O(1) 
    def merge(self, leftHull, rightHull):
        # Find leftmost point of left hull and rightmost point of right hull
        # Time Complexity: O(n)| Space Complexity: O(1) 
        leftMostPoint = leftHull.index(max(leftHull, key=lambda leftPoint: leftPoint.x()))
        rightMostPoint = rightHull.index(min(rightHull, key=lambda rightPoint: rightPoint.x()))

        # Find upper and lower tangent between the two hulls
        # Time Complexity: O(n)| Space Complexity: O(1) 
        upperTangent = self.find_upper_tangent(leftMostPoint, rightMostPoint, leftHull, rightHull)
        lowerTangent = self.find_lower_tangent(leftMostPoint, rightMostPoint, leftHull, rightHull)

        # Initialize merged hull 
        mergedHull = []

        upperTangentLeft = upperTangent[0]
        upperTangentRight = upperTangent[1]
        
        lowerTangetLeft = lowerTangent[0]
        lowerTangetRight = lowerTangent[1]

        # Add lowest point on left hull to merged hull
        mergedHull.append(leftHull[lowerTangetLeft])

        # Add remaining left hull below upper tangent
        # Time Complexity: O(n)| Space Complexity: O(1) 
        while lowerTangetLeft != upperTangentLeft:
            lowerTangetLeft = (lowerTangetLeft+1) % len(leftHull)
            mergedHull.append(leftHull[lowerTangetLeft])
        
        # Add point from right hull on upper tangent
        mergedHull.append(rightHull[upperTangentRight])

        # Add remaining right hull above lower tangent
        # Time Complexity: O(n)| Space Complexity: O(1) 
        while upperTangentRight != lowerTangetRight:
            upperTangentRight = (upperTangentRight+1) % len(rightHull)
            mergedHull.append(rightHull[upperTangentRight])

        return mergedHull
    
    # This function finds upper tangent between two convex hulls
    # Time Complexity: O(n) | Space Complexity: O(1) 
    def find_upper_tangent(self,  leftMostPoint, rightMostPoint, leftHull, rightHull):
        leftIndex, rightIndex = leftMostPoint, rightMostPoint
        
        # Calculate initial slope between points
        # Time Complexity: O(1) | Space Complexity: O(1) 
        slope = (rightHull[rightIndex].y() - leftHull[leftIndex].y()) / (rightHull[rightIndex].x() - leftHull[leftIndex].x())

        # Iteratively update indices to find max upward slope  
        # Time Complexity: O(n) | Space Complexity: O(1) 
        leftTurned = True
        while leftTurned:
            leftTurned = False
            while True:   
                slope2 = (rightHull[rightIndex].y() - leftHull[(leftIndex-1) % len(leftHull)].y())/(rightHull[rightIndex].x() - leftHull[(leftIndex-1) % len(leftHull)].x())
                if slope2 < slope:  
                    leftTurned = True
                    slope = slope2
                    leftIndex = (leftIndex-1) % len(leftHull)
                else:
                    break
            while True:   
                slope2 = (rightHull[(rightIndex+1) % len(rightHull)].y() - leftHull[leftIndex].y())/(rightHull[(rightIndex+1) % len(rightHull)].x() - leftHull[leftIndex].x())
                if slope2 > slope:  
                    leftTurned = True
                    slope = slope2
                    rightIndex = (rightIndex+1) % len(rightHull)
                else:
                    break

        return (leftIndex, rightIndex)

    # This function finds lower tangent between two convex hulls
    # Time Complexity: O(n) | Space Complexity: O(1) 
    def find_lower_tangent(self, leftMostPoint, rightMostPoint, leftHull, rightHull):
        leftIndex, rightIndex = leftMostPoint, rightMostPoint
            
        # Calculate initial slope between points
        # Time Complexity: O(1) | Space Complexity: O(1) 
        slope = (rightHull[rightIndex].y() - leftHull[leftIndex].y()) / (rightHull[rightIndex].x() - leftHull[leftIndex].x())

        # Iteratively update indices to find max downward slope  
        # Time Complexity: O(n) | Space Complexity: O(1) 
        rightTurned = True
        while rightTurned:
            rightTurned = False
            while True:  
                slope2 = (rightHull[(rightIndex-1) % len(rightHull)].y() - leftHull[leftIndex].y())/(rightHull[(rightIndex-1) % len(rightHull)].x() - leftHull[leftIndex].x())
                if slope2 < slope:   
                    rightTurned = True
                    slope = slope2
                    rightIndex = (rightIndex-1) % len(rightHull)
                else:
                    break
            while True:   
                slope2 = (rightHull[rightIndex].y() - leftHull[(leftIndex+1) % len(leftHull)].y())/(rightHull[rightIndex].x() - leftHull[(leftIndex+1) % len(leftHull)].x())
                if slope2 > slope:  
                    rightTurned = True
                    slope = slope2
                    leftIndex = (leftIndex+1) % len(leftHull)
                else:
                    break

        return (leftIndex, rightIndex)  
    
    # Displays convex hulls and tangents recursively for visualization
    # Time complexity is O(n) | Space complexity is O(n) 
    def show_recursion_button(self, leftHull, rightHull, upper, lower): 
        # Display upper and lower tangent lines
        # Time complexity is O(1) | Space complexity is O(1) 
        self.showTangent([QLineF(leftHull[upper[0]], rightHull[upper[1]]), QLineF(leftHull[lower[0]], rightHull[lower[1]])], BLUE)

        # Display left hull 
        # Time complexity is O(n) | Space complexity is O(n) 
        self.showHull([QLineF(leftHull[i], leftHull[(i+1) % len(leftHull)]) for i in range(len(leftHull))], RED)

        # Display right hull
        # Time complexity is O(n) | Space complexity is O(n) 
        self.showHull([QLineF(rightHull[i], rightHull[(i+1) % len(rightHull)]) for i in range(len(rightHull))], GREEN)

        # Erase left hull 
        # Time complexity is O(n) | Space complexity is O(n) 
        self.eraseHull([QLineF(leftHull[i], leftHull[(i+1) % len(leftHull)]) for i in range(len(leftHull))])

        # Erase right hull
        # Time complexity is O(n) | Space complexity is O(n) 
        self.eraseHull([QLineF(rightHull[i], rightHull[(i+1) % len(rightHull)]) for i in range(len(rightHull))])

        # Erase tangent lines
        # Time complexity is O(1) | Space complexity is O(1) 
        self.eraseTangent([QLineF(leftHull[upper[0]], rightHull[upper[1]]), QLineF(leftHull[lower[0]], rightHull[lower[1]])])