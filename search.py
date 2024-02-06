# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
import copy
from util import PriorityQueue


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def goalTest(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
        Given a state, returns available actions.
        Returns a list of actions
        """        
        util.raiseNotDefined()

    def getResult(self, state, action):
        """
        Given a state and an action, returns resulting state.
        """
        util.raiseNotDefined()

    def getCost(self, state, action):
        """
        Given a state and an action, returns step cost, which is the incremental cost 
        of moving to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

class Node:
    """
    Search node object for your convenience.

    This object uses the state of the node to compare equality and for its hash function,
    so you can use it in things like sets and priority queues if you want those structures
    to use the state for comparison.

    Example usage:
    >>> S = Node("Start", None, None, 0)
    >>> A1 = Node("A", S, "Up", 4)
    >>> B1 = Node("B", S, "Down", 3)
    >>> B2 = Node("B", A1, "Left", 6)
    >>> B1 == B2
    True
    >>> A1 == B2
    False
    >>> node_list1 = [B1, B2]
    >>> B1 in node_list1
    True
    >>> A1 in node_list1
    False
    """
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __hash__(self):
        return hash(self.state)

    def __eq__(self, other):
        return self.state == other.state

    def __ne__(self, other):
        return self.state != other.state


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def breadthFirstSearch(problem):
    """
    Breadth-First Search algorithm to find a solution to the given problem.

    Args:
    - problem: The problem instance to be solved.

    Returns:
    - A list of actions to reach the goal state, or an empty list if no solution is found.
    """
    def getSuccessors(state):
        """
        Get the successor states, actions, and costs for a given state.

        Args:
        - state: The current state.

        Returns:
        - List of successor states with corresponding actions and costs.
        """
        successors = []
        actions = problem.getActions(state)
        for action in actions:
            next_state = problem.getResult(state, action)
            cost = problem.getCost(state, action)
            successors.append((next_state, action, cost))
        return successors

    # Initialize the starting state
    initial_node = {'state': problem.getStartState(), 'cost': 0}

    # Check if the starting state is already a goal state
    if problem.goalTest(initial_node['state']):
        return []

    # Initialize the frontier using a queue
    frontier = util.Queue()
    frontier.push((initial_node['state'], []))

    # Keep track of explored states
    explored = set()

    while True:
        # If the frontier is empty, the search has failed
        if frontier.isEmpty():
            raise Exception("Search failed")

        # Get the current node to explore
        current_node = frontier.pop()

        # Get the successor nodes
        successors = getSuccessors(current_node[0])

        # Mark the current node as explored
        explored.add(current_node[0])

        # Loop through each successor node
        for successor in successors:
            # Check if the node is not already explored
            if successor[0] not in explored:
                # Check if the node is the goal state
                if problem.goalTest(successor[0]):
                    # Return the actions to reach the goal
                    current_node[1].append(successor[1])
                    return current_node[1]
                else:
                    # Update the actions to reach the current node
                    actions = list(current_node[1])
                    actions.append(successor[1])
                    # Add child nodes to the frontier
                    frontier.push((successor[0], actions))

    
def depthFirstSearch(problem): 

    "*** YOUR CODE HERE ***"   
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth. Begin with a depth of 1 and increment depth by 1 at every step.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.goalTest(problem.getStartState()))
    print("Actions from start state:", problem.getActions(problem.getStartState()))

    Then try to print the resulting state for one of those actions
    by calling problem.getResult(problem.getStartState(), one_of_the_actions)
    or the resulting cost for one of these actions
    by calling problem.getCost(problem.getStartState(), one_of_the_actions)

    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
    
def UniformCostSearch(problem):
    """Search the node that has the lowest path cost first."""
    "*** YOUR CODE HERE ***"  
    util.raiseNotDefined()
    



def aStarSearch(problem, heuristic=nullHeuristic):
    """
    A* Search algorithm to find a solution to the given problem.

    Args:
    - problem: The problem instance to be solved.
    - heuristic: The heuristic function for estimating the cost to reach the goal.

    Returns:
    - A list of actions to reach the goal state, or an empty list if no solution is found.
    """

    def getSuccessors(state):
        """
        Retrieve the successor states, actions, and costs for a given state.

        Args:
        - state: The current state.

        Returns:
        - List of successor states with corresponding actions and costs.
        """
        successors = []
        actions = problem.getActions(state)
        for action in actions:
            next_state = problem.getResult(state, action)
            cost = problem.getCost(state, action)
            successors.append((next_state, action, cost))
        return successors

    # Get the starting state
    initial_state = {'state': problem.getStartState(), 'cost': 0}

    # Check if the starting state is already a goal state
    if problem.goalTest(initial_state['state']):
        return []

    # Initialize the frontier using a priority queue
    search_frontier = PriorityQueue()

    # Push the starting node to the frontier with the combined cost (f(n) = g(n) + h(n))
    search_frontier.push((initial_state['state'], initial_state['cost'], 0, []), heuristic(initial_state['state'], problem))

    # Keep track of explored states
    explored_states = set()

    while True:
        # Get the node to be searched
        current_node = search_frontier.pop()

        # Get the children of the node
        successors = getSuccessors(current_node[0])

        # Add the current node to the set of explored nodes
        explored_states.add(current_node[0])

        for successor in successors:
            # Check if the child node is not already explored
            if successor[0] not in explored_states:
                # Check if the goal node is reached
                if problem.goalTest(successor[0]):
                    current_node[3].append(successor[1])
                    return current_node[3]  # Return the actions to reach the goal
                else:
                    # Append actions to reach the current node
                    actions_to_current = list(current_node[3])
                    actions_to_current.append(successor[1])
                    # Calculate f(n) = g(n) + h(n)
                    total_cost = problem.getCostOfActions(actions_to_current) + heuristic(successor[0], problem) + successor[2]
                    # Push the child node to the frontier with the combined cost
                    search_frontier.push((successor[0], successor[2], total_cost, actions_to_current), total_cost)


# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
