# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util, math

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]



### ----------- testing with AIMA code --------- ####################

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        "Create a search tree Node, derived from a parent by an action."
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth  = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def solution(self):
        "Return the sequence of actions to go from the root to this node."
        return [node.action for node in self.path()[1:]]

    def path(self):
        "Return a list of nodes forming the path from the root to this node."
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)


def depthFirstSearch(problem):
    node = Node(problem.getStartState())
    frontier = util.Stack()
    frontier.push(node)
    if problem.isGoalState(node.state):
        return node
    explored = set()
    while not(frontier.isEmpty()):
        node = frontier.pop()
        explored.add(node.state)
        node_list = []
        for i in problem.getSuccessors(node.state):
            new_node = Node(i[0], node, i[1], i[2] )
            node_list.append(new_node)
        for child in node_list:
            if child.state not in explored and child not in frontier.list:
                if problem.isGoalState(child.state):
                    return child.solution()
                frontier.push(child)


def breadthFirstSearch(problem):
    node = Node(problem.getStartState())
    frontier = util.Queue()
    frontier.push(node)
    if problem.isGoalState(node.state):
        return node
    explored = set()
    while not(frontier.isEmpty()):
        node = frontier.pop()
        explored.add(node.state)
        node_list = []
        for i in problem.getSuccessors(node.state):
            new_node = Node(i[0], node, i[1], i[2] )
            node_list.append(new_node)
        for child in node_list:
            if child.state not in explored and child not in frontier.list:
                if problem.isGoalState(child.state):
                    return child.solution()
                frontier.push(child)


def uniformCostSearch(problem): #best_first_graph_search(problem, f):

    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.

    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""

    node = Node(problem.getStartState())
    frontier = util.PriorityQueue()
    frontier.push(node, 0)

    if problem.isGoalState(node.state):
        return node

    explored = set()
    
    while not (frontier.isEmpty()):
        node = frontier.pop()
        if problem.isGoalState(node.state):
            return node.solution()
        explored.add(node.state)
        node_list = []
        for i in problem.getSuccessors(node.state):
            priority = node.path_cost + i[2] 
            new_node = Node(i[0], node, i[1], priority )
            node_list.append(new_node)
        
        for child in node_list:
            if child.state not in explored and child not in frontier.heap:
                frontier.push(child, child.path_cost)
            elif child in frontier.heap:
                index = 0
                for i in len(frontier.heap):
                    if frontier.heap[i].state == child.state:
                        index = i
                        break
                incumbent = frontier.heap[index]
                if child.path_cost < incumbent.path_cost:
                    #frontier.heap.remove(incumbent)
                    frontier.append(child, child.path_cost)
                    

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"


    initial_state = problem.getStartState()
    node = Node(initial_state, None, None, heuristic(initial_state, problem))
    frontier = util.PriorityQueue()
    #print 'heuristic(initial_state, problem): ', heuristic(initial_state, problem)
    frontier.push(node, 0 + heuristic(initial_state, problem))

    if problem.isGoalState(node.state):
        return node

    explored = set()

    cost_so_far = 0
    temp = 0
    while not (frontier.isEmpty()):
        
        node = frontier.pop()#[1]
        cost_so_far = node.path_cost - heuristic(node.state, problem)

        if problem.isGoalState(node.state):
            return node.solution()

        explored.add(node.state)
        node_list = []

        for i in problem.getSuccessors(node.state):
            priority = cost_so_far + i[2] + heuristic(i[0], problem)
            new_node = Node(i[0], node, i[1], priority )
            node_list.append(new_node)
            if new_node.path_cost - node.path_cost < 0:
                print 'SOS!!!!!!!!!!!!!!!!!!!!!!!!'

        for child in node_list:
            if child.state not in explored and child not in frontier.heap:
                frontier.push(child, child.path_cost)
            elif child in frontier.heap:
                index = 0
                for i in len(frontier.heap):
                    if frontier.heap[i].state == child.state:
                        index = i
                        break                
                incumbent = frontier.heap[index]
                if child.path_cost < incumbent.path_cost:
                    #frontier.heap.remove(incumbent)
                    frontier.append(child, child.path_cost)                    
    
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
