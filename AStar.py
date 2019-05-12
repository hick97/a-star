import numpy as np
from math import inf
import sys
import re

IS_NEIGHBOR = 1

class AStar:
    def __init__(self, names, distanceMatrix, adjacencyMatrix, initialPoint, endPoint):
        self.distanceMatrix = distanceMatrix
        self.names = names
        self.adjacencyMatrix = adjacencyMatrix
        self.initialNode = initialPoint
        self.endNode = endPoint
        self.route = []
        self.final_cost = 0

        self.route.append(vertices_names[initialPoint])

        self.execution()

    def getRoute(self):
        return self.route

    def getFinalCost(self):
        return self.final_cost

    def execution(self):
        current_node = self.initialNode
        accumulated_dist = 0

        while(current_node != self.endNode):
            print('Current node: ' + vertices_names[current_node])

            search_border = self.checkBorder(current_node)
            best_cost = inf
            accumulated_dist_local = 0
            node_to_be_visited = 0

            for node in search_border:
                cost = self.cost(current_node, node, accumulated_dist)

                if(cost[0] <= best_cost):
                    best_cost = cost[0]
                    accumulated_dist_local = cost[1]
                    node_to_be_visited = node

            accumulated_dist += accumulated_dist_local
            current_node = node_to_be_visited

            print('Im going to node: ' + vertices_names[current_node])
            print('Local cost was: ' + str(best_cost))
            self.route.append(vertices_names[current_node])

        self.final_cost = best_cost

    def checkBorder(self, node):
        border = []
        for index, value in enumerate(self.adjacencyMatrix[node]):
            if value == IS_NEIGHBOR:
                border.append(index)
        return border

    def cost(self, current_node, next_node, accumulated_dist=0):
        g_n = accumulated_dist + self.distanceMatrix[current_node][next_node]
        h_n = self.distanceMatrix[self.endNode][next_node]
        f_n = g_n + h_n

        return f_n, g_n


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: python AStar.py <graph_data> <start> <end>')
        sys.exit(1)

    matrix_dist = []
    matrix_adj = []
    matrix_length = 0

    with open(sys.argv[1], 'r') as file:
        text = file.readlines()
        vertices_count = int(text[0])
        vertices_names = text[1].split()

        start = vertices_names.index(sys.argv[2])
        end = vertices_names.index(sys.argv[3])

        for index, line in enumerate(text[2:2+vertices_count]):
            adj = [int(a) for a in re.split(' ', line)]
            matrix_adj.append(adj)

        for index, line in enumerate(text[2+vertices_count:]):
            dist = [int(d) for d in re.split(' ', line)]
            matrix_dist.append(dist)

        search = AStar(vertices_names, matrix_dist, matrix_adj, start, end)

        print('Final cost: ' + str(search.getFinalCost()))
        print('Route: ' + str(search.getRoute()))