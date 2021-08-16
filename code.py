import sys
from builtins import print
from itertools import permutations

from api import *
from time import sleep
import numpy as np
import random as r
from math import sqrt, fabs

#######    YOUR CODE FROM HERE #######################
import random
grid =[]
neigh=[[-1,-1],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1]]

FLT_MAX = 1000000000.0

class Node:
	def __init__(self,value,point):
		self.value = value  #0 for blocked,1 for unblocked
		self.point = point
		self.parent = None
		self.move=None
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
	return pt[0]>=0 and pt[1]>=0 and pt[0]<200 and pt[1]<200


def is_des(p, des, rad):
	if des[0] <= p[0] < des[0]+rad and des[1] <= p[1] < des[1]+rad:
		return True
	return False

def neighbours(point):  #returns valid neighbours
	global grid,neigh
	x,y = point.point
	links=[]
	for i in range(len(neigh)):
		newX=x+neigh[i][0]
		newY=y+neigh[i][1]
		if not isValid((newX,newY)):
			continue
		links.append((i+1,grid[newX][newY]))
	return links


def get_map():
	green_list = get_greenZone_list()
	back_list = get_obstacles_list()
	red_list = get_redZone_list()
	for ele in green_list:
		tl = ele[0]
		br = ele[2]
		for x in range(tl[0], br[0]+1):
			for y in range(tl[1], br[1]+1):
				zoo[x][y].color = Cell.Green
	for ele in back_list:
		tl = ele[0]
		br = ele[2]
		for x in range(tl[0], br[0]+1):
			for y in range(tl[1], br[1]+1):
				zoo[x][y].color = Cell.Black
	for ele in red_list:
		tl = ele[0]
		br = ele[2]
		for x in range(tl[0], br[0]+1):
			for y in range(tl[1], br[1]+1):
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
	return dx + dy + (sqrt(2) - 2)*min(dx, dy)


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
			_p = [p[0]+ele[0], p[1]+ele[1]]
			if isValid(_p):
				if is_des(_p, end, end_radius):
					zoo[_p[0]][_p[1]].parent = p
					return _p
				else:
					if is_close[_p[0]][_p[1]] == False and zoo[_p[0]][_p[1]].color != Cell.Black:
						if index % 2 == 0:
							_g = zoo[p[0]][p[1]].g + 1.414
						else:
							_g = zoo[p[0]][p[1]].g + 1
						_h = cal_h_val(_p, end)
						_f = _g + _h
						if zoo[_p[0]][_p[1]].f > _f:
							open_list.append([_p[0], _p[1]])
							zoo[_p[0]][_p[1]].g = _g
							zoo[_p[0]][_p[1]].h = _h
							zoo[_p[0]][_p[1]].f = _f
							zoo[_p[0]][_p[1]].parent = p
	return [0, 0]


def get_min_path(graph):
	num = len(graph)
	perms = permutations([x for x in range(1, num)])
	min_dis = FLT_MAX
	for ele in perms:
		dis = graph[0][ele[0]]
		for index in range(1, len(ele)):
			dis += graph[ele[index-1]][ele[index]]
		if dis < min_dis:
			ret = ele
			min_dis = dis
	return [0]+list(ret)


########## Default Level 1 ##########
def level1(botId):
	des_list, rad_list = get_map()
	print(des_list, rad_list)
	bot_list = get_botPose_list()
	end = a_star(bot_list[0], des_list[0], rad_list[0])
	path = trace_path(end)
	while len(path) != 0:
		step = path.pop()
		successful_move, mission_complete = send_command(0, step)
		# if successful_move:
		# 	print("YES")
		# else:
		# 	print("NO")
		if mission_complete:
			print("MISSION COMPLETE")
			break
		pos = get_botPose_list()
		# print(pos[0])



def level2(botId):
	des_list, rad_list = get_map()
	pos = get_botPose_list()
	print(pos)
	nodes = pos + des_list
	rad_list = [0] + rad_list
	print(nodes)
	dis = list()
	for ele1 in nodes:
		_dis = list()
		for ele2 in nodes:
			_dis.append((ele1[0] - ele2[0])*(ele1[0] - ele2[0]) + (ele1[1] - ele2[1])*(ele1[1] - ele2[1]))
		dis.append(_dis)
	print(dis)
	path = get_min_path(dis)
	print(path)
	begin = nodes[0]
	for index in range(1, len(path)):
		end = a_star(begin, nodes[path[index]], rad_list[path[index]])
		steps = trace_path(end)
		reset_zoo()
		begin = end
		while len(steps) != 0:
			step = steps.pop()
			successful_move, mission_complete = send_command(0, step)
			if mission_complete:
				print("MISSION COMPLETE")
				return



def level3(botId):
	pass

def level4(botId):
	pass

def level5(botId):
	pass

def level6(botId):
	pass


#######    DON'T EDIT ANYTHING BELOW  #######################

if  __name__=="__main__":
	botId = int(sys.argv[1])
	level = get_level()
	if level == 1:
		level1(botId)
	elif level == 2:
		level2(botId)
	elif level == 3:
		level3(botId)
	elif level == 4:
		level4(botId)
	elif level == 5:
		level5(botId)
	elif level == 6:
		level6(botId)
	else:
		print("Wrong level! Please restart and select correct level")

