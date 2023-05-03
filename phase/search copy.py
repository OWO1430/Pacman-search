# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
# Modified by Shang-Tse Chen (stchen@csie.ntu.edu.tw) on 03/03/2022

"""
This is the main entry point for HW1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)
import heapq

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "fast": fast,
    }.get(searchMethod)(maze)

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    # start = maze.getStart()
    # print(start)
    # queue = []
    # isUsed = {}
    # queue.append([start])
    # while queue:
    #     # print(queue)
    #     path = queue.pop(0)
    #     pos = path[-1]
    #     # print(pos)
    #     neighbors = maze.getNeighbors(pos[0], pos[1])
    #     if pos == maze.getObjectives()[0]:
    #         # print(path)
    #         return path
    #     if (pos not in isUsed.keys()):
    #         isUsed[pos] = True
    #         for neighbor in neighbors:
    #             if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isUsed.keys():
    #                 newPath = list(path)
    #                 newPath.append(neighbor)
    #                 #print(newPath)
    #                 queue.append(newPath)
    # return []
    start = maze.getStart()
    print(start)
    queue = []
    isUsed = {}
    queue.append([start])
    while queue:
        # print(queue)
        path = queue.pop(0)
        pos = path[-1]
        if (pos not in isUsed.keys()):
            isUsed[pos] = True
        # print(pos)
        neighbors = maze.getNeighbors(pos[0], pos[1])
        if pos == maze.getObjectives()[0]:
            # print(path)
            return path

        for neighbor in neighbors:
            if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isUsed[pos]:
                isUsed[pos].append(neighbor)
                newPath = list(path)
                newPath.append(neighbor)
                #print(newPath)
                queue.append(newPath)
    return []


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    def m_distance(row, col):
        goal = maze.getObjectives()[0]
        return abs(row - goal[0]) + abs(col - goal[1])
    
    start = maze.getStart()
    goal = maze.getObjectives()[0]
    print(m_distance(start[0], start[1]))
    queue = []
    result = []
    isUsed = {}
    heapq.heappush(queue, (m_distance(start[0], start[1]), [start]))

    while queue:
        current_set = heapq.heappop(queue)
        path = current_set[1]
        pos = path[-1]
        neighbors = maze.getNeighbors(pos[0], pos[1])
        if pos == maze.getObjectives()[0]:
            heapq.heappush(result ,current_set)
            continue
        elif result and result[0][0] < len(path):
            break
        if (pos not in isUsed.keys()):
            isUsed[pos] = True
            for neighbor in neighbors:
                if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isUsed.keys():
                    newPath = list(path)
                    newPath.append(neighbor)
                    heapq.heappush(queue,(len(newPath)-1+m_distance(neighbor[0], neighbor[1]),newPath))
    final = result[0]
    return final[1]
def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # # TODO: Write your code here
    final_goals = maze.getObjectives()

    def m_distance(row, col, goal):
        return abs(row - goal[0]) + abs(col - goal[1])
    
    def Max_distance(state, goals):
        dis = 0
        for goal in goals:
            if(m_distance(state[0], state[1], goal)>dis):
                dis = m_distance(state[0], state[1], goal)
        return dis

    start = maze.getStart()
    queue = []
    result = []
    isUsed = {}
    heapq.heappush(queue, (Max_distance(start, final_goals), [start]), final_goals)
    while queue:
        current_set = heapq.heappop(queue)
        path = current_set[1]
        pos = path[-1]
        if (pos not in isUsed.keys()):
            isUsed[pos] = []
        neighbors = maze.getNeighbors(pos[0], pos[1])
        if pos == maze.getObjectives()[0]:
            
            continue
        elif result and result[0][0] < len(path):
            break
        for neighbor in neighbors:
            if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isUsed[pos]:
                isUsed[pos].append(neighbor)
                newPath = list(path)
                newPath.append(neighbor)
                heapq.heappush(queue,(len(newPath)-1+m_distance(neighbor[0], neighbor[1]),newPath))
    final = result[0]
    return final[1]

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here




def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
