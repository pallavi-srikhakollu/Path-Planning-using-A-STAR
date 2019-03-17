#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig,ax = plt.subplots()
path = []
class Grid():
    """A node class for A* Pathfinding"""

    def __init__(self, root=None, grid=None):
        self.root = root
        self.grid = (grid[0],grid[1])

        self.start_dist = 0
        self.end_dist = 0
        self.h = self.start_dist + self.end_dist

    #def __eq__(self, other):
    #    return self.grid[0] == other.grid[0] and self.

    def __cmp__(self, other):
        return cmp(self.h, other.h)

mid_point = (9,10)


def get_grid_pos(point):
	return (int(point[0]+9),int(point[1]+10))
     
def is_diagonal(point1,point2):
	x1,y1 = point1
	x,y = point2
	if ((x1 == x-1 and y1 == y-1) or (x1 == x+1 and y1 == y-1) or (x1 == x-1 and y1 == y-1) or (x1 == x+1 and y1 == y+1)):
		return True
	else:
		return False

def check_diagonal(point1,point2):
	x1,y1 = point1
	x,y = point2

	if (x1 == x-1 and y1 == y-1) :
		#print("Diagonal element: first condition ")
		if y-1 > 0 and x-1 > 0:
			if map[y-1][x] == 1 and map[y][x-1] == 1:
				return  False
		
	elif (x1 == x+1 and y1 == y-1) :
		#print("Diagonal element: second condition ")
		if y-1 > 0 and x+1 < 18:
			if map[y-1][x] == 1 and map[y][x+1] == 1:
				return  False

	elif (x1 == x-1 and y1 == y-1) :
		#print("Diagonal element: third condition ")
		if y+1 < 20 and x-1 > 0:
			if map[y][x-1] == 1 and map[y+1][x] == 1:
				return  False
	
	elif (x1 == x+1 and y1 == y+1):
		#print("Diagonal element: fourth condition ")
		if y+1 < 20 and x+1 < 18:
			if map[y+1][x] == 1 and map[y][x+1] == 1:
				return  False
	
	else:	
		return True

map = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]
map = np.flipud(map)

def get_Path(goal_x,goal_y):
	global path
	visited_nodes = []
	no_path_nodes = []
	closed_list = []

	#start_node = (0,0,0,0)
	start_point = (-8.0,-2.0)
	end_point = (goal_x,goal_y)
	#(4.5,9.0)

	start_node = Grid(None,get_grid_pos(start_point))
	plt.scatter(start_node.grid[0],start_node.grid[1],marker='^',facecolor = 'none',edgecolor = 'red')
	print("starting_node",start_node.grid[0],start_node.grid[1])
	#Grid(None,(1,5))
	goal_node = Grid(None,get_grid_pos(end_point))
	plt.scatter(goal_node.grid[0],goal_node.grid[1],marker='^',facecolor = 'none',edgecolor = 'blue')
	print("goal_node",goal_node.grid[0],goal_node.grid[1])
	print()

	#Grid(None,(16, 10))
	#(14,3)

	index = 0 
	visited_nodes.append(start_node)
	reached = 0


	while len(visited_nodes) > 0:
		
		print("Came here")

		if reached > 0:
			break

		index = visited_nodes.index(min(visited_nodes, key=lambda x: x.h))

		node = visited_nodes[index]
		
		x = node.grid[0]
		y = node.grid[1] 
		print("visited_node top",x,y)
		neighbours = []
		closed_list.append(node)
		#print("Before length")
		visited_nodes.pop(index)

		for i in range(-1,2):
			for j in range(-1,2):
				#if x+i < 18 and x+i > 0 and y+i < 4 and y+i > 0:s
				if x+i == x and y+j == y:
					print("")
				elif x+i < 18 and x+i > -1 and y+j < 20 and y+j > -1:
					neighbours.append((x+i,y+j))


		#print("Neighbours",neighbours)

		for n in neighbours:
			x1,y1 = n
			#xs,ys = start_node.grid
			xg,yg = goal_node.grid
			
			
			if map[y1][x1] == 1:
				print("obstacle found")
				no_path_nodes.append(n)
				
				continue
				
			
			else:
				if is_diagonal((x1,y1),(x,y)):
					#print("Diagonal node")
					if check_diagonal((x1,y1),(x,y)):
					 	
					 	dig_node = Grid(None,(x1,y1))
					 	closed_list.append(dig_node)
					 	no_path_nodes.append(n)
					 	#continue
				
				else:
					
					dist_s = node.start_dist + ((x1-x) ** 2) + ((y1-y) ** 2)
					dist_g = ((x1-xg) ** 2) + ((y1-yg) ** 2)


					found = 1
					closed_found = 1

					for node2 in closed_list:
						#print("Node in closed: ",node2.grid[0],node2.grid[1],x1,y1)
						if node2.grid[0] == x1 and node2.grid[1] == y1:
							#print("In closed list should skip",node2.grid[0],node2.grid[1])
							closed_found += 1
					#continue
					if closed_found > 1:
						continue

					#print("Visited_nodes for loop",x1,y1)
					for node1 in visited_nodes:
						#print("Visited_nodes for loop",node2.grid[0],node2.grid[1])
						if node1.grid[0] == x1 and node1.grid[1] == y1:
							#print("Node found in visited")
							if node1.start_dist > dist_s:
								print("Root updation for ", node1.grid[0],node1.grid[1])
								node1.root = node
								node1.start_dist = dist_s
								node1.end_dist = dist_g
								node1.h = dist_s + dist_g
						 	
							found = found + 1

					if found == 1 : # Not present in openlist

						node3 =  Grid(node,(x1,y1))
						node3.start_dist = dist_s
						node3.end_dist = dist_g
						
						node3.h = dist_s + dist_g
							
						visited_nodes.append(node3)

					if x1 == goal_node.grid[0] and y1 == goal_node.grid[1]:
						reached += 1
						print("Goal reached")
						print("Path is:")
						

						current = node
						while current is not None:
							path.append(current.grid)
							print(current.grid[0],current.grid[1])
							plt.scatter(current.grid[0]+1,current.grid[1]+1,marker='o',facecolor = 'none',edgecolor = 'yellow') 
							current = current.root
							
					
		print("Nodes in vistor list")
		
		#for nodet in visited_nodes:
			#print("node:",nodet.grid[0],nodet.grid[1],"Cost:",nodet.h) 	
	
		
	for i in range(0,18):
	  	for j in range(0,20):
	  		if map[j][i] == 1:
	  			#plt.scatter(i+1,20-j,marker='o',facecolor = 'none',edgecolor = 'green') 
	  			plt.scatter(i+1,j+1,marker='o',facecolor = 'none',edgecolor = 'green')    	
					
	#print(len(visited_nodes))
	plt.savefig("astar_plot1.png",dpi=300)
	path1 = []	
	for i in range(len(path)-1,0,-1):
		path1.append(path[i])
	#path =  path.reverse()
	return path1

path1 = get_Path(4.5,9.0)
#print("Path1",path1)






