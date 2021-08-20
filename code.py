import copy
import sys
from builtins import print
from itertools import permutations
from math import sqrt

from api import *

#######    YOUR CODE FROM HERE #######################
grid = []
neigh = [[-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]

FLT_MAX = 1000000000.0


class Node:
    def __init__(self, value, point):
        self.value = value  # 0 for blocked,1 for unblocked
        self.point = point
        self.parent = None
        self.move = None
        self.H = 0
        self.G = 0


class Cell:
    White = 0
    Black = 1
    Green = 2
    Red = 3

    def __init__(self):
        self.parent = None
        self.color = Cell.White
        self.g = FLT_MAX
        self.h = FLT_MAX
        self.f = FLT_MAX


zoo = [[Cell() for x in range(200)] for y in range(200)]


def isValid(pt):
    return pt[0] >= 0 and pt[1] >= 0 and pt[0] < 200 and pt[1] < 200


def is_des(p, des, rad):
    if des[0] <= p[0] < des[0] + rad and des[1] <= p[1] < des[1] + rad:
        return True
    return False


def neighbours(point):  # returns valid neighbours
    global grid, neigh
    x, y = point.point
    links = []
    for i in range(len(neigh)):
        newX = x + neigh[i][0]
        newY = y + neigh[i][1]
        if not isValid((newX, newY)):
            continue
        links.append((i + 1, grid[newX][newY]))
    return links


def get_map():
    green_list = get_greenZone_list()
    back_list = get_obstacles_list()
    red_list = get_redZone_list()
    for ele in green_list:
        tl = ele[0]
        br = ele[2]
        for x in range(tl[0], br[0] + 1):
            for y in range(tl[1], br[1] + 1):
                zoo[x][y].color = Cell.Green
    for ele in back_list:
        tl = ele[0]
        br = ele[2]
        for x in range(tl[0], br[0] + 1):
            for y in range(tl[1], br[1] + 1):
                zoo[x][y].color = Cell.Black
    for ele in red_list:
        tl = ele[0]
        br = ele[2]
        for x in range(tl[0], br[0] + 1):
            for y in range(tl[1], br[1] + 1):
                zoo[x][y].color = Cell.Red
    return [ele[0] for ele in green_list], [ele[2][0] - ele[0][0] for ele in green_list]


def trace_path(des):
    x = des[0]
    y = des[1]
    steps = list()
    while zoo[x][y].parent[0] != x or zoo[x][y].parent[1] != y:
        parent = zoo[x][y].parent
        _x = x - parent[0]
        _y = y - parent[1]
        if _x == -1:
            steps.append(2 + _y)
        elif _x == 0:
            if _y == 1:
                steps.append(4)
            else:
                steps.append(8)
        else:
            steps.append(6 - _y)
        x = parent[0]
        y = parent[1]
    return steps


def reset_zoo():
    for row in zoo:
        for cell in row:
            cell.parent = None
            cell.g = FLT_MAX
            cell.h = FLT_MAX
            cell.f = FLT_MAX


def cal_h_val(pt, des):
    dx = abs(pt[0] - des[0])
    dy = abs(pt[1] - des[1])
    return dx + dy + (sqrt(2) - 2) * min(dx, dy)


def a_star(begin, end, end_radius):
    zoo[begin[0]][begin[1]].g = 0
    zoo[begin[0]][begin[1]].h = 0
    zoo[begin[0]][begin[1]].f = 0
    zoo[begin[0]][begin[1]].parent = begin
    is_close = [[False for x in range(200)] for y in range(200)]
    open_list = list()
    open_list.append(begin)
    while len(open_list) != 0:
        p = open_list.pop(0)
        is_close[p[0]][p[1]] = True
        for index, ele in enumerate(neigh):
            _p = [p[0] + ele[0], p[1] + ele[1]]
            if isValid(_p):
                if is_des(_p, end, end_radius):
                    zoo[_p[0]][_p[1]].parent = p
                    return _p
                else:
                    if is_close[_p[0]][_p[1]] == False and zoo[_p[0]][_p[1]].color != Cell.Black:
                        if index % 2 == 0:
                            _g = 1.414
                        else:
                            _g = + 1
                        if zoo[_p[0]][_p[1]].color == Cell.Red:
                            _g *= 2
                        _g += zoo[p[0]][p[1]].g
                        _h = cal_h_val(_p, end)
                        _f = _g + _h
                        if zoo[_p[0]][_p[1]].f > _f:
                            open_list.append([_p[0], _p[1]])
                            zoo[_p[0]][_p[1]].g = _g
                            zoo[_p[0]][_p[1]].h = _h
                            zoo[_p[0]][_p[1]].f = _f
                            zoo[_p[0]][_p[1]].parent = p
    return [0, 0]


def get_min_path(bot_pos, green_pos):
    nodes = [bot_pos] + green_pos
    dist = list()
    for ele1 in nodes:
        _dist = list()
        for ele2 in nodes:
            _dist.append((ele1[0] - ele2[0]) * (ele1[0] - ele2[0]) + (ele1[1] - ele2[1]) * (ele1[1] - ele2[1]))
        dist.append(_dist)
    perms = permutations([x for x in range(1, len(nodes))])
    min_val = FLT_MAX
    for perm in perms:
        val = dist[0][perm[0]]
        for index in range(1, len(perm)):
            val += dist[perm[index-1]][perm[index]]
        if val < min_val:
            min_val = val
            ret = perm
    return [x - 1 for x in ret]


def get_zone(ls_bot_pos, ls_green_pos):
    ret = [list() for _ in range(len(ls_bot_pos))]
    for green_idx, green in enumerate(ls_green_pos):
        min_val = FLT_MAX
        for bot_idx, bot in enumerate(ls_bot_pos):
            val = (bot[0] - green[0]) * (bot[0] - green[0]) + (bot[1] - green[1]) * (bot[1] - green[1])
            if val < min_val:
                min_val = val
                nearest = bot_idx
        ret[nearest].append(green_idx)
    return ret


########## Default Level 1 ##########
def level1(bot_id):
    des_list, rad_list = get_map()
    bot_list = get_botPose_list()
    end = a_star(bot_list[0], des_list[0], rad_list[0])
    path = trace_path(end)
    while len(path) != 0:
        step = path.pop()
        successful_move, mission_complete = send_command(bot_id, step)
        if mission_complete:
            print("MISSION COMPLETE")
            return


def level2(bot_id):
    ls_green, ls_rad = get_map()
    ls_pos = get_botPose_list()
    path = get_min_path(ls_pos[0], ls_green)
    nodes = [ls_pos[0]]
    rad_nodes = [0]
    for ele in path:
        nodes.append(ls_green[ele])
        rad_nodes.append(ls_rad[ele])
    begin = nodes[0]
    for index in range(1, len(nodes)):
        end = a_star(begin, nodes[index], rad_nodes[index])
        steps = trace_path(end)
        reset_zoo()
        begin = end
        while len(steps) != 0:
            step = steps.pop()
            successful_move, mission_complete = send_command(bot_id, step)
            if mission_complete:
                print("MISSION COMPLETE")
                return


def level3(bot_id):
    ls_green, ls_rad = get_map()
    ls_pos = get_botPose_list()
    zones = get_zone(ls_pos, ls_green)
    print(zones)
    ls_pos = [ls_pos[bot_id]]
    ls_green = [ls_green[x] for x in zones[bot_id]]
    ls_rad = [ls_rad[x] for x in zones[bot_id]]
    path = get_min_path(ls_pos[0], ls_green)
    nodes = [ls_pos[0]]
    rad_nodes = [0]
    for ele in path:
        nodes.append(ls_green[ele])
        rad_nodes.append(ls_rad[ele])
    begin = nodes[0]
    for index in range(1, len(nodes)):
        end = a_star(begin, nodes[index], rad_nodes[index])
        steps = trace_path(end)
        reset_zoo()
        begin = end
        while len(steps) != 0:
            step = steps.pop()
            successful_move, mission_complete = send_command(bot_id, step)
            if mission_complete:
                print("MISSION COMPLETE")
                return


def level4(bot_id):
    ls_green, ls_rad = get_map()
    ls_pos = get_botPose_list()
    zones = get_zone(ls_pos, ls_green)
    print(zones)
    ls_pos = [ls_pos[bot_id]]
    ls_green = [ls_green[x] for x in zones[bot_id]]
    ls_rad = [ls_rad[x] for x in zones[bot_id]]
    path = get_min_path(ls_pos[0], ls_green)
    nodes = [ls_pos[0]]
    rad_nodes = [0]
    for ele in path:
        nodes.append(ls_green[ele])
        rad_nodes.append(ls_rad[ele])
    begin = nodes[0]
    for index in range(1, len(nodes)):
        end = a_star(begin, nodes[index], rad_nodes[index])
        steps = trace_path(end)
        reset_zoo()
        begin = end
        while len(steps) != 0:
            step = steps.pop()
            successful_move, mission_complete = send_command(bot_id, step)
            if mission_complete:
                print("MISSION COMPLETE")
                return


def level5(bot_id):
    ls_green, ls_rad = get_map()
    ls_pos = get_botPose_list()
    zones = get_zone(ls_pos, ls_green)
    print(zones)
    ls_pos = [ls_pos[bot_id]]
    ls_green = [ls_green[x] for x in zones[bot_id]]
    ls_rad = [ls_rad[x] for x in zones[bot_id]]
    path = get_min_path(ls_pos[0], ls_green)
    nodes = [ls_pos[0]]
    rad_nodes = [0]
    for ele in path:
        nodes.append(ls_green[ele])
        rad_nodes.append(ls_rad[ele])
    begin = nodes[0]
    for index in range(1, len(nodes)):
        end = a_star(begin, nodes[index], rad_nodes[index])
        steps = trace_path(end)
        reset_zoo()
        begin = end
        while len(steps) != 0:
            step = steps.pop()
            successful_move, mission_complete = send_command(bot_id, step)
            if mission_complete:
                print("MISSION COMPLETE")
                return


def level6(bot_id):
    ls_green, ls_rad = get_map()
    ls_pos = get_botPose_list()
    zones = get_zone(ls_pos, ls_green)
    print(zones)
    ls_pos = [ls_pos[bot_id]]
    ls_green = [ls_green[x] for x in zones[bot_id]]
    ls_rad = [ls_rad[x] for x in zones[bot_id]]
    path = get_min_path(ls_pos[0], ls_green)
    nodes = [ls_pos[0]]
    rad_nodes = [0]
    for ele in path:
        nodes.append(ls_green[ele])
        rad_nodes.append(ls_rad[ele])
    begin = nodes[0]
    for index in range(1, len(nodes)):
        end = a_star(begin, nodes[index], rad_nodes[index])
        steps = trace_path(end)
        reset_zoo()
        begin = end
        while len(steps) != 0:
            step = steps.pop()
            successful_move, mission_complete = send_command(bot_id, step)
            if mission_complete:
                print("MISSION COMPLETE")
                return


#######    DON'T EDIT ANYTHING BELOW  #######################
if __name__ == "__main__":
    idx = int(sys.argv[1])
    level = get_level()
    if level == 1:
        level1(idx)
    elif level == 2:
        level2(idx)
    elif level == 3:
        level3(idx)
    elif level == 4:
        level4(idx)
    elif level == 5:
        level5(idx)
    elif level == 6:
        level6(idx)
    else:
        print("Wrong level! Please restart and select correct level")
