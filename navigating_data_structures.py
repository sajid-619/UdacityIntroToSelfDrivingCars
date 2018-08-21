# coding: utf-8

# # Implementing a Route Planner
# In this project you will use A\* search to implement a "Google-maps" style route planning algorithm.

# ## The Map

# In[1]:


# Run this cell first!

from helpers import Map, load_map_10, load_map_40, show_map
import math

get_ipython().run_line_magic('load_ext', 'autoreload')
get_ipython().run_line_magic('autoreload', '2')


# ### Map Basics

# In[2]:


map_10 = load_map_10()
show_map(map_10)


# The map above (run the code cell if you don't see it) shows a disconnected network of 10 intersections. The two intersections on the left are connected to each other but they are not connected to the rest of the road network. This map is quite literal in its expression of distance and connectivity. On the graph above, the edge between 2 nodes(intersections) represents a literal straight road not just an abstract connection of 2 cities.
# 
# These `Map` objects have two properties you will want to use to implement A\* search: `intersections` and `roads`
# 
# **Intersections**
# 
# The `intersections` are represented as a dictionary. 
# 
# In this example, there are 10 intersections, each identified by an x,y coordinate. The coordinates are listed below. You can hover over each dot in the map above to see the intersection number.

# In[3]:


map_10.intersections


# **Roads**
# 
# The `roads` property is a list where `roads[i]` contains a list of the intersections that intersection `i` connects to.

# In[4]:


# this shows that intersection 0 connects to intersections 7, 6, and 5
map_10.roads[0] 


# In[5]:


# This shows the full connectivity of the map
map_10.roads


# In[6]:


# map_40 is a bigger map than map_10
map_40 = load_map_40()
show_map(map_40)


# ### Advanced Visualizations
# 
# The map above shows a network of roads which spans 40 different intersections (labeled 0 through 39). 
# 
# The `show_map` function which generated this map also takes a few optional parameters which might be useful for visualizaing the output of the search algorithm you will write.
# 
# * `start` - The "start" node for the search algorithm.
# * `goal`  - The "goal" node.
# * `path`  - An array of integers which corresponds to a valid sequence of intersection visits on the map.

# In[7]:


# run this code, note the effect of including the optional
# parameters in the function call.
show_map(map_40, start=5, goal=34, path=[5,16,37,12,34])


# ## The Algorithm
# ### Writing your algorithm
# The algorithm written will be responsible for generating a `path` like the one passed into `show_map` above. In fact, when called with the same map, start and goal, as above you algorithm should produce the path `[5, 16, 37, 12, 34]`. However you must complete several methods before it will work.
# 
# ```bash
# > PathPlanner(map_40, 5, 34).path
# [5, 16, 37, 12, 34]
# ```

# In[8]:


# Do not change this cell
# When you write your methods correctly this cell will execute
# without problems
class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
        
    def get_path(self):
        """ Reconstructs path after search """
        if self.path:
            return self.path 
        else :
            self.run_search()
            return self.path
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tenative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False


# Create the following methods:

# In[9]:


def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # TODO: return a data structure suitable to hold the set of nodes already evaluated
    return set()


# In[10]:


def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
        # TODO: return a data structure suitable to hold the set of currently discovered nodes 
        # that are not evaluated yet. Make sure to include the start node.
        create_set = set()
        create_set.add(self.start)
        return create_set
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")


# In[11]:


def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    # TODO: return a data structure that shows which node can most efficiently be reached from another,
    # for each node.
    return {}


# In[12]:


def create_gScore(self):
    """Creates and returns a data structure that holds the cost of getting from the start node to that node, for each node.
    The cost of going from start to start is zero."""
    # TODO:  a data structure that holds the cost of getting from the start node to that node, for each node.
    # for each node. The cost of going from start to start is zero. The rest of the node's values should be set to infinity.
    gScore = {}
    for i in range (len(self.map.intersections)):
        gScore[i] = float('inf')
    gScore[self.start] = 0
    return gScore


# In[13]:


def create_fScore(self):
    """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    # TODO:  a data structure that holds the total cost of getting from the start node to the goal
    # by passing by that node, for each node. That value is partly known, partly heuristic.
    # For the first node, that value is completely heuristic. The rest of the node's value should be 
    # set to infinity.
    fScore = {}
    for i in range(len(self.map.intersections)):
        fScore[i] = float('inf')
    fScore[self.start] = self.heuristic_cost_estimate(self.start)
    return fScore


# In[14]:


def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    # TODO: Set map to new value.
    self.map = M
    return None


# In[15]:


def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    # TODO: Set start value. Remember to remove goal, closedSet, openSet, cameFrom, gScore, fScore, 
    # and path attributes' values.
    self.start=start
    self.goal = None
    self.closedSet = None
    self.openSet = None
    self.cameFrom = None
    self.gScore = None
    self.fScore = None
    self.path=None
    return None
    


# In[16]:


def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    # TODO: Set goal value.
    self.goal = goal
    return None


# In[17]:


def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    # TODO: Return the node in the open set with the lowest value of f(node).
    minValue_node = {}
    for node in self.openSet:
        minValue_node[node] = self.calculate_fscore(node)
    minValue_key = min(minValue_node,key = minValue_node.get)
    
    return minValue_key


# In[18]:


def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    # TODO: Return the neighbors of a node
    return set(self.map.roads[node])


# In[19]:


def get_gScore(self, node):
    """Returns the g Score of a node"""
    # TODO: Return the g Score of a node
    return self.gScore.get(node, 0.0)


# In[20]:


def get_tenative_gScore(self, current, neighbor):
    """Returns the tenative g Score of a node"""
    # TODO: Return the g Score of the current node 
    # plus distance from the current node to it's neighbors
    return (self.get_gScore(current) + self.distance(current,neighbor))


# In[21]:


def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    # TODO: Return True if the open set is empty. False otherwise.
    if len(self.openSet) == 0:
        return True
    else:
        return False


# In[22]:


def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    # TODO: Compute and return the Euclidean L2 Distance
    return math.sqrt(((self.map.intersections[node_1][0] - self.map.intersections[node_2][0])**2)+((self.map.intersections[node_1][1] - self.map.intersections[node_2][1])**2))


# In[23]:


def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    # TODO: Return the heuristic cost estimate of a node
    return self.distance(node,self.goal)


# In[24]:


def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    # TODO: Calculate and returns the f score of a node. 
    # REMEMBER F = G + H
    return (self.get_gScore(node) + self.heuristic_cost_estimate(node))


# In[25]:


def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    # TODO: Record the best path to a node, by updating cameFrom, gScore, and fScore
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tenative_gScore(current,neighbor)
    self.fScore[current] = self.calculate_fscore(current)
    return None


# In[26]:


def _reset(self):
    """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore andpath attributes"""
    self.closedSet = None
    self.openSetSet = None
    self.cameFrom = None
    self.gScore = None
    self.fScore = None
    self.path = self.run_search() if self.map and self.start and self.goal else None


# In[2]:


PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner._reset = _reset
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.get_tenative_gScore = get_tenative_gScore
PathPlanner.is_open_empty = is_open_empty
PathPlanner.distance = distance
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to


# In[28]:


planner = PathPlanner(map_40, 5, 34)
path = planner.path
if path == [5, 16, 37, 12, 34]:
    print("great! Your code works for these inputs!")
else:
    print("something is off, your code produced the following:")
    print(path)


# ### Testing your Code
# If the code below produces no errors, your algorithm is behaving correctly. You are almost ready to submit! Before you submit, go through the following submission checklist:
# 
# **Submission Checklist**
# 
# 1. Does my code pass all tests?
# 2. Does my code implement `A*` search and not some other search algorithm?
# 3. Do I use an **admissible heuristic** to direct search efforts towards the goal?
# 4. Do I use data structures which avoid unnecessarily slow lookups?
# 
# When you can answer "yes" to all of these questions, submit by pressing the Submit button in the lower right!

# In[1]:


from test import test

test(PathPlanner)


# ## Questions
# 
# **Instructions**  Answer the following questions in your own words. We do not you expect you to know all of this knowledge on the top of your head. We expect you to do research and ask question. However do not merely copy and paste the answer from a google or stackoverflow. Read the information and understand it first. Then use your own words to explain the answer.

# - How would you explain A-Star to a family member(layman)?
# 
# ** ANSWER **: The A* algorithm combines features of uniform-cost search and pure heuristic search to efficiently compute optimal solutions. A* algorithm is a best-first search algorithm in which the cost associated with a node is f(n) = g(n) + h(n), where g(n) is the cost of the path from the initial state to node n and h(n) is the heuristic estimate or the cost or a path from node n to a goal. Thus, f(n) estimates the lowest total cost of any solution path going through node n. At each point a node with lowest f value is chosen for expansion. Ties among nodes of equal f value should be broken in favor of nodes with lower h values. The algorithm terminates when a goal is chosen for expansion.
# 
# A* algorithm guides an optimal path to a goal if the heuristic function h(n) is admissible, meaning it never overestimates actual cost. For example, since airline distance never overestimates actual highway distance, and manhatten distance never overestimates actual moves in the gliding tile.

# - How does A-Star search algorithm differ from Uniform cost search? What about Best First search?
# 
# ** ANSWER **:1. Uniform cost search expands the node with lowest path cost whereas best expand the node with closest to the goal.
# 
# 2.Uniform cost search can not deal with heuristic function whereas best first search can deal with heuristic function.
# 
# 3. In uniform cost search f(n) = g(n) whereas In best first search f(n) = h(n)

# - What is a heuristic?
# 
# ** ANSWER **: In computer science, artificial intelligence, and mathematical optimization, a heuristic  is a technique designed for solving a problem more quickly when classic methods are too slow, or for finding an approximate solution when classic methods fail to find any exact solution.

# - What is a consistent heuristic?
# 
# ** ANSWER **:In the study of path-finding problems in artificial intelligence, a heuristic function is said to be consistent if its estimate is always less than or equal to the estimated distance from any neighboring vertex to the goal, plus the step cost of reaching that neighbor.
# 
# In an equation, it would look like this: C(n, n’) + h(n’) ≤ h(n)

# - What is a admissible heuristic? 
# 
# ** ANSWER **:In computer science, specifically in algorithms related to pathfinding, a heuristic function is said to be admissible if it never overestimates the cost of reaching the goal, i.e. the cost it estimates to reach the goal is not higher than the lowest possible cost from the current point in the path.
# 
# 

# - ___ admissible heuristic are consistent.
# *CHOOSE ONE*
#     - All
#     - Some
#     - None
#     
# ** ANSWER **: Some

# - ___ Consistent heuristic are admissible.
# *CHOOSE ONE*
#     - All
#     - Some
#     - None
#     
# ** ANSWER **: All
