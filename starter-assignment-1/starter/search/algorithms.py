import heapq
import math

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
    
    def get_h(self, goal):
        xf = goal.get_x()
        yf = goal.get_y()
        difx = abs(xf-self._x)
        dify = abs(xf-self._y)

        return 1.5* min(difx, dify) + abs(difx - dify)


class Dijkstra:
    def __init__(self, gridmap):
        self.__map = gridmap
        self.__open = []
        self.__closed = {}
        self.__numExpanded = 0

    def get_closed_data(self):
        return self.__closed


    def djikstra(self, start, goal):
        self.__open.append(start)
        self.__closed[start.state_hash()] = start
        while self.__open:
            curState = heapq.heappop(self.__open)
            self.__numExpanded += 1
            if curState == goal:
                return self.getPath(self.__closed[goal.state_hash()], start), self.__closed[goal.state_hash()].get_cost(), self.__numExpanded
            children = self.__map.successors(curState)
            for child in children:
                hashVal = child.state_hash()    
                if hashVal not in self.__closed:
                    self.__closed[hashVal] = child
                    child.set_cost(child.get_g())
                    child.set_parent(curState)
                    heapq.heappush(self.__open, child)
                # can it happen where the child is poped but then revisited later?, as in was the lowest cost so expanded but then a later 1 came back to it with cheaper -A: no this is not possible and can be proven, it is expected that when a stated is expanded that it holds its optimal cost from the start
                elif child.get_g() < self.__closed[hashVal].get_cost():
                    # does this work, since the open list holds a reference and not the object, so it would have duplicates of the same position if we just set closed to child
                    assert self.__closed[hashVal] == child
                    self.__closed[hashVal].set_cost(child.get_g())
                    self.__closed[hashVal].set_g(child.get_g())
                    self.__closed[hashVal].set_parent(curState)
                    heapq.heapify(self.__open)

        return None, -1, self.__numExpanded
    

    def getPath(self, end, start):
        parent = end
        path = [end]
        while True:
            parent = parent.get_parent()
            if parent == start:
                path.append(parent)
                break
            path.append(parent)
        
        return path[::-1]




class AStar:
    def __init__(self, gridmap):
        self.__map = gridmap
        self.__open = []
        self.__closed = {}
        self.__numExpanded = 0

    def astar(self, start, goal):
        self.__open.append(start)
        self.__closed[start.state_hash()] = start
        while self.__open:
            curState = heapq.heappop(self.__open)
            self.__numExpanded += 1
            if curState == goal:
                return self.getPath(self.__closed[goal.state_hash()],start), self.__closed[goal.state_hash()].get_cost(), self.__numExpanded
            children = self.__map.successors(curState)
            for child in children:
                hashVal = child.state_hash()    
                if hashVal not in self.__closed:
                    self.__closed[hashVal] = child
                    child.set_cost(child.get_g()+child.get_h(goal)) # put in octile heuristic
                    child.set_parent(curState)
                    heapq.heappush(self.__open, child)
                # can it happen where the child is poped but then revisited later?, as in was the lowest cost so expanded but then a later 1 came back to it with cheaper -A: no this is not possible and can be proven, it is expected that when a stated is expanded that it holds its optimal cost from the start
                elif child.get_g() + child.get_h(goal) < self.__closed[hashVal].get_cost():
                    assert self.__closed[hashVal] == child
                    # does this work, since the open list holds a reference and not the object, so it would have duplicates of the same position if we just set closed to child
                    self.__closed[hashVal].set_cost(child.get_g()+child.get_h(goal)) # update so that it is the heuristic
                    # since these should have the same x and y they have the same h and as such only need to update the g
                    self.__closed[hashVal].set_g(child.get_g())
                    self.__closed[hashVal].set_parent(curState)
                    heapq.heapify(self.__open)

        return None, -1, self.__numExpanded
    

    def getPath(self, end, start):
        parent = end
        path = [end]
        while True:
            parent = parent.get_parent()
            if parent == start:
                path.append(parent)
                break
            path.append(parent)
        return path[::-1]