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
    print(start)
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
            if(m_distance(state[0], state[1], goal)>dis):
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
        game_state = (pos, tuple(current_goal))
        # if game_state in isUsed.keys():
        #     continue
        current_cost = current_set[1]
        neighbors = maze.getNeighbors(pos[0], pos[1])

        if not current_goal:
            return path
            break
        if pos in current_goal:
            current_goal.remove(pos)


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
                heapq.heappush(queue,(new_cost+Max_distance(start, final_goals),new_cost,newPath, current_goal))

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    start = maze.getStart()
    goals = maze.getObjectives()
    paths = []
    p_queue = q.PriorityQueue()

    for i in range(len(goals)):
        for j in range(i+1, len(goals)):
            paths.append((i, j))

    len_map = {}
    for path in paths:
        c_maze = deepcopy(maze)
        c_maze.setStart(goals[path[0]])
        c_maze.setObjectives([goals[path[1]]])
        dist = len(astar(c_maze))
        len_map[path] = dist-1

    # a map of the previous or parent nodes
    prev = {(start, tuple(goals)):None}

    # the priority queue gets (f, distance to current node(g), (current node, remaing goals))
    s_node = (mst_heuristic(start, tuple(goals), len_map, goals)+0, 0, (start, tuple(goals)))
    p_queue.put(s_node)

    cur_node_dst_map = {s_node[2]:0}

    while p_queue:
        cur = p_queue.get()
        cur_pos = cur[2][0]
        if (len(cur[2][1]) == 0):
            return getPath(cur[2], prev)

        neighbors = maze.getNeighbors(cur_pos[0], cur_pos[1])
        for n in neighbors:
            goals_from_node = tuple(goals_to_get(n, cur[2][1]))
            dst_node = (n, goals_from_node)
            if dst_node in cur_node_dst_map and cur_node_dst_map[dst_node] <= cur_node_dst_map[cur[2]]+1:
                continue
            #update distance map of the node
            cur_node_dst_map[dst_node] = cur_node_dst_map[cur[2]]+1
            #update node's parent
            prev[dst_node] = cur[2]

            #update p_queue: this part is borrowed from the textbook Fig. 3.26
            old_f = cur[0]
            new_f = cur_node_dst_map[dst_node]+mst_heuristic(n, goals_from_node, len_map, goals)
            new_f = max(old_f, new_f)

            new_node = (new_f, cur_node_dst_map[dst_node], dst_node)
            p_queue.put(new_node)



def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
