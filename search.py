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
import time # delete later

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

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    """
    create a stack(Fringe)
    create a list of visited
    create a map of child -> parent

    add initial to the stack

    while(!empty):
        node = stack.pop
        check if isgoalState(node)
        add node to visited
        for succ in successors if succ not in visitors:
            stack.push(succ)
    """
    fringe = util.Stack()
    visited = []
    paths = {}

    initial = problem.getStartState()
    fringe.push(initial)
    paths[initial] = []

    while not fringe.isEmpty():
        node = fringe.pop()
        if problem.isGoalState(node):
            return paths[node]
        visited.append(node)
        for succ in problem.getSuccessors(node):
            if succ[0] not in visited:
                fringe.push(succ[0])
                paths[succ[0]] = paths[node] + [succ[1]]

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first.

    fringe = util.Queue()
    paths = {}

    initial = problem.getStartState()
    fringe.push(initial)
    paths[initial] = []

    visited = {initial}

    while not fringe.isEmpty():
        node = fringe.pop()
        if problem.isGoalState(node):
            return paths[node]

        # time.sleep(0.5)
        # print('')
        # print('CurrentState: ', node)
        visited.add(node)

        # print('State is already visited: ', s)
        for succ in problem.getSuccessors(node):
            # print(succ[0], ' is ', succ[0] in visited, ' visited already')
            if succ[0] not in visited:
                fringe.push(succ[0])
                paths[succ[0]] = paths[node] + [succ[1]]

        print('Visited: ', sorted(visited))
        print('Fringe: ', sorted(fringe.list))

    util.raiseNotDefined()
    """
    """Search the shallowest nodes in the search tree first."""

    fringe = util.Queue()
    paths = {}

    initial = problem.getStartState()
    fringe.push(initial)
    paths[initial] = []
    visited = {initial}

    while not fringe.isEmpty():
        node = fringe.pop()
        if problem.isGoalState(node):
            return paths[node]

        for succ in problem.getSuccessors(node):
            if succ[0] not in visited:
                visited.add(succ[0])
                fringe.push(succ[0])
                paths[succ[0]] = paths[node] + [succ[1]]

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    fringe = util.PriorityQueue()
    paths = {}
    costs = {}

    initial = problem.getStartState()
    costs[initial] = 0
    fringe.push(initial, costs[initial])
    paths[initial] = []
    visited = {initial}


    while not fringe.isEmpty():
        node = fringe.pop()
        visited.add(node)


        if problem.isGoalState(node):
            return paths[node]



        for succ in problem.getSuccessors(node):
            if succ[0] not in visited:
                newCost = costs[node] + succ[2]

                if succ[0] in costs:
                    """
                    UPDATE: make sure if a path to a node has already been found,
                            we should only update that path in the future if the new path
                            has a lower cost
                    """
                    if newCost < costs[succ[0]]:
                        paths[succ[0]] = paths[node] + [succ[1]]
                        costs[succ[0]] = newCost
                else:
                    paths[succ[0]] = paths[node] + [succ[1]]
                    costs[succ[0]] = newCost

                fringe.update(succ[0], costs[succ[0]])

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    fringe = util.PriorityQueue()
    paths = {}
    pathCosts = {}

    initial = problem.getStartState()
    pathCosts[initial] = 0
    fringe.push(initial, heuristic(initial, problem))
    paths[initial] = []
    visited = {initial}


    while not fringe.isEmpty():
        node = fringe.pop()
        visited.add(node)

        if problem.isGoalState(node):
            return paths[node]

        for succ in problem.getSuccessors(node):
            if succ[0] not in visited:
                newPathCost = pathCosts[node] + succ[2]

                if succ[0] in pathCosts:
                    """
                    UPDATE: make sure if a path to a node has already been found,
                            we should only update that path in the future if the new path
                            has a lower cost
                    """
                    if newPathCost < pathCosts[succ[0]]:
                        paths[succ[0]] = paths[node] + [succ[1]]
                        pathCosts[succ[0]] = newPathCost
                else:
                    paths[succ[0]] = paths[node] + [succ[1]]
                    pathCosts[succ[0]] = newPathCost

                fringe.update(succ[0], pathCosts[succ[0]] + heuristic(succ[0], problem))

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
