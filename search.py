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
    start = maze.getStart()
    # print(start)
    queue = []
    isUsed = {}
    queue.append([start])
    while queue:
        # print(queue)
        path = queue.pop(0)
        pos = path[-1]
        # print(pos)
        neighbors = maze.getNeighbors(pos[0], pos[1])
        if pos == maze.getObjectives()[0]:
            # print(path)
            return path
        if (pos not in isUsed.keys()):
            isUsed[pos] = True
            for neighbor in neighbors:
                if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isUsed.keys():
                    newPath = list(path)
                    newPath.append(neighbor)
                    #print(newPath)
                    queue.append(newPath)



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
    # print(m_distance(start[0], start[1]))
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
            return path

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
            # dis = dis + m_distance(state[0], state[1], goal)
            if m_distance(state[0], state[1], goal) > dis:
                dis = m_distance(state[0], state[1], goal)
        # print(dis)
        return dis

    start = maze.getStart()
    queue = []
    result = []
    isUsed = {}
    heapq.heappush(queue, (Max_distance(start, final_goals),0,[start], maze.getObjectives()))
    while queue:
        current_set = heapq.heappop(queue)
        path = current_set[2]
        pos = path[-1]
        current_goal= list(current_set[3])
        game_state = (pos, len(tuple(current_goal)))
        # if game_state in isUsed.keys():
        #     continue
        current_cost = current_set[1]
        
        if pos in current_goal:
            current_goal.remove(pos)
        if not current_goal:
            print(path)
            return path
            break
        neighbors = maze.getNeighbors(pos[0], pos[1])


        # if game_state not in isUsed.keys():
        #     isUsed[game_state] = True
        #     for neighbor in neighbors:
        #         if maze.isValidMove(neighbor[0], neighbor[1]) == True and (neighbor, tuple(current_goal)) not in isUsed.keys():
        #             new_cost = current_cost+1
        #             newPath = path + [neighbor]
        #             heapq.heappush(queue,(new_cost+Max_distance(start, final_goals),new_cost,newPath, current_goal))
        
        for neighbor in neighbors:
            neighbor_state = (neighbor, tuple(current_goal))
            if maze.isValidMove(neighbor[0], neighbor[1]) == True and (neighbor_state not in isUsed.keys()):
                isUsed[neighbor_state] = True
                new_cost = current_cost+1
                newPath = path + [neighbor]
                heapq.heappush(queue,(new_cost+Max_distance(neighbor, current_goal),new_cost,newPath, current_goal))

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    def bfs_dis(_from, _to):
        q = []
        isVisited = {}
        q.append([_from])
        while q:
            # print(queue)
            path = q.pop(0)
            pos = path[-1]
            # print(pos)
            neighbors = maze.getNeighbors(pos[0], pos[1])
            if pos == _to:
                return path
            if (pos not in isVisited.keys()):
                isVisited[pos] = True
                for neighbor in neighbors:
                    if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isVisited.keys():
                        newPath = list(path)
                        newPath.append(neighbor)
                        #print(newPath)
                        q.append(newPath)
    def Mst_cost(_dots, _mst_list):
        mst_cost = 0
        isVisited = {}
        isVisited[_dots[0]] = True

        while len(_dots) > len(isVisited):
            q = []
            for i in isVisited:
                for j in _dots:
                    if isVisited.get(j) != True:
                        heapq.heappush(q, (_mst_list[(i, j)], j))
            tmp = heapq.heappop(q) 
            isVisited[tmp[1]] = True
            mst_cost = mst_cost + tmp[0]
        # print(mst_cost)
        return mst_cost
    

    start = maze.getStart()
    final_goals = maze.getObjectives()

    dots = maze.getObjectives()
    # dots = [start] + n_dots

    edge = {}
    mst_list = {}

    # for i in dots:
    #     for j in dots:
    #         if i != j:
    #             path = bfs_dis(i, j)
    #             edge[(i, j)] = path
    #             mst_list[(i, j)] = len(path)
    for i in range(len(dots)):
        for j in range(i, len(dots)):
                path = bfs_dis(dots[i], dots[j])
                edge[(dots[i], dots[j])] = path
                edge[(dots[j], dots[i])] = path
                mst_list[(dots[i], dots[j])] = len(path)
                mst_list[(dots[j], dots[i])] = len(path)

    start = maze.getStart()
    queue = []
    isUsed = {}
    heapq.heappush(queue, (Mst_cost(dots, mst_list),0,[start], maze.getObjectives()))
    while queue:
        current_set = heapq.heappop(queue)
        # print(current_set)
        path = current_set[2]
        pos = path[-1]
        current_goal= list(current_set[3])
        game_state = (pos, tuple(current_goal))
        current_cost = current_set[1]
        neighbors = maze.getNeighbors(pos[0], pos[1])

        if pos in current_goal:
            current_goal.remove(pos)
        if not current_goal:
            return path
            break

        for neighbor in neighbors:
            neighbor_state = (neighbor, tuple(current_goal))
            if maze.isValidMove(neighbor[0], neighbor[1]) == True and (neighbor_state not in isUsed.keys()):
                isUsed[neighbor_state] = True
                # new_goal = [pos] + current_goal
                new_cost = current_cost+1
                newPath = path + [neighbor]
                heapq.heappush(queue,(new_cost+Mst_cost(current_goal, mst_list),new_cost,newPath, current_goal))



def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    
    start = maze.getStart()
    final_goals = maze.getObjectives()
    final_path = []
    
    while final_goals:
        q = []
        q.append([start])
        isVisited = {}
        while q:
            path = q.pop(0)
            pos = path[-1]
            # print(pos)
            neighbors = maze.getNeighbors(pos[0], pos[1])
            if pos in final_goals:
                final_goals.remove(pos)
                path.pop(0)
                final_path = final_path + path
                start = pos
                break
            if (pos not in isVisited.keys()):
                isVisited[pos] = True
                for neighbor in neighbors:
                    if maze.isValidMove(neighbor[0], neighbor[1]) == True and neighbor not in isVisited.keys():
                        newPath = list(path)
                        newPath.append(neighbor)
                        #print(newPath)
                        q.append(newPath)
    final_path = [maze.getStart()] + final_path
    # print(final_path)
    return final_path
