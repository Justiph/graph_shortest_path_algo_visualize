import pygame
from maze import SearchSpace
from const import *

def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()

    raise NotImplementedError('not implemented')

def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')

    open_set = [g.start]
    closed_set = []
    father = [-1]*g.get_length()

    while open_set:
        current_node = open_set.pop(0)
        
        current_node.set_color(YELLOW, sc)
        
        if g.is_goal(current_node):
            #print("Goal reached! Generating path...")
            generate_path(g, current_node, father, sc)
            return
        
        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            if neighbor not in open_set and neighbor not in closed_set:
                open_set.append(neighbor)
                father[neighbor.id] = current_node.id
                neighbor.set_color(RED, sc)
                
        
        closed_set.append(current_node)
        current_node.set_color(BLUE, sc)
    
    print("Cannot reach goal.")
    return
    #raise NotImplementedError('not implemented')



def DIJKSTRA(g: SearchSpace, sc: pygame.Surface):
    dist = [1e7] * g.get_length()
    dist[g.start.id] = 0
    nodes = list(g.grid_cells)
    father = [-1]*g.get_length()
    
    while nodes:
        current_node = min(nodes, key=lambda node: dist[node.id])
        current_node.set_color(YELLOW, sc)
        nodes.remove(current_node)
        
        if g.is_goal(current_node):
            generate_path(g, current_node, father, sc)
            return
        
        neighbors = g.get_neighbors(current_node)
        for neighbor in neighbors:
            tentative_distance = dist[current_node.id] + 1
            
            if tentative_distance < dist[neighbor.id] and neighbor in nodes:
                neighbor.set_color(RED, sc)
                dist[neighbor.id] = tentative_distance
                father[neighbor.id] = current_node.id
        current_node.set_color(BLUE, sc)
    print("Cannot reach goal.")
    return       

    
    


def mid_point(node):
    j, i = node.id%COLS, node.id//COLS
    x, y = j*(A+A1)+BOUND, i*(A+A1)+BOUND
    return x + A/2, y + A/2

def generate_path(g: SearchSpace, goal_node, father, sc: pygame.Surface):
    current_node = goal_node
    current_node.set_color(PURPLE, sc)
    while father[current_node.id] != -1:
        father_node = g.grid_cells[father[current_node.id]]
        
        if father_node.id == g.start.id:
            father_node.set_color(ORANGE, sc)
            
        mid1 = mid_point(current_node)
        mid2 = mid_point(father_node)
        
        pygame.draw.line(sc, WHITE, mid1, mid2)
        pygame.display.update()
        pygame.time.delay(5)
        
        
        current_node = father_node
        
        