import heapq
import math
from map import Map

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost
    


class Djikstra:
    def __init__(self, gridmap:Map):
        self.__map = gridmap
        self.__open = []
        self.__closed = {}
        self.__numExpanded = 0


    def djikstra(self, start:State, goal:State):
        self.__open.append(start)
        self.__closed[start.state_hash()] = start
        while self.__open:
            curState = heapq.heappop(self.__open)
            self.__numExpanded += 1
            if curState == goal:
                return self.getPath(), goal.get_g(), self.__numExpanded
            children = self.__map.successors(curState)
            for child in children:
                hashVal = child.state_hash()    
                if hashVal not in self.__closed:
                    self.__closed[hashVal] = child
                    child.set_parent(curState)
                    heapq.heappush(self.__open, child)
                elif child.get_g() < self.__closed[hashVal].get_g():
                    # does this work, since the open list holds a reference and not the object, so it would have duplicates of the same position if we just set closed to child
                    self.__closed[hashVal].set_g(child.get_g())
                    self.__closed[hashVal].set_parent(curState)
                    heapq.heapify(self.__open)

        return None, -1, self.__numExpanded
    

    def getPath(self, end:State):
        parent = end
        pathE2S =  [end]
        path = []
        while True:
            parent = parent.get_parent()
            if parent == None:
                break
            pathE2S.append(parent)

        while pathE2S:
            path.append(pathE2S.pop())




class Astar:
    def __init__(self):
        pass
    def astar(self):
        pass