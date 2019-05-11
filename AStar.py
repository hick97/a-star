import numpy as np
import sys
import re

IS_NEIGHBOR = 1


class AStar:
    def __init__(self, distanceMatrix, adjacencyMatrix, initialPoint, endPoint):
        self.distanceMatrix = distanceMatrix
        self. adjacencyMatrix = adjacencyMatrix
        self.initialNode = initialPoint
        self.endNode = endPoint
        self.route = []
        self.final_cost = 0

        self.route.append(initialPoint)

        self.execution()

    def getRoute(self):
        return self.route

    def getFinalCost(self):
        return self.final_cost

    def execution(self):
        current_node = self.initialNode
        accumulated_dist = 0

        while(current_node != self.endNode):
            print('Current node: ' + str(current_node))

            current_neighbors = self.checkNeighbors(current_node)
            initial_fitness = self.fitness(
                current_neighbors[0], accumulated_dist)
            initial_d0 = initial_fitness[1]
            best_fitness = initial_fitness[0]

            better_than_initial = False
            accumulated_dist_local = 0
            node_to_be_visited = 0

            for index, value in enumerate(current_neighbors):
                if index != 0:
                    f_result = self.fitness(value, accumulated_dist)

                    if(f_result[0] <= best_fitness):
                        best_fitness = f_result[0]
                        accumulated_dist_local = f_result[1]
                        current_node = value
                        better_than_initial = True
                        node_to_be_visited = value

            if(not better_than_initial):
                accumulated_dist += initial_d0
                current_node = current_neighbors[0]
                node_to_be_visited = current_neighbors[0]
            else:
                accumulated_dist += accumulated_dist_local

            print('Local cost was: ' + str(best_fitness))
            self.final_cost += best_fitness
            self.route.append(node_to_be_visited)

            print('Im going to node: ' + str(current_node))

    def checkNeighbors(self, node):
        neighbors = []
        for index, value in enumerate(self.adjacencyMatrix[node]):
            if(int(value) == IS_NEIGHBOR):
                neighbors.append(index)
        return neighbors

    def fitness(self, current_node, accumulated_dist=0):

        d0_n = accumulated_dist + \
            int(self.distanceMatrix[self.initialNode][current_node])
        h_n = int(self.distanceMatrix[self.endNode][current_node])

        f_node = int(d0_n + h_n)

        return f_node, d0_n


if __name__ == '__main__':

    if len(sys.argv) != 3:
        print('Usage: python AStar.py distmatrix.txt adjmatrix.txt')
        sys.exit(1)

    matrix_dist = []
    matrix_adj = []
    matrix_length = 0

    file = open(sys.argv[1], 'r')
    text = file.readlines()
    for index, line in enumerate(text):
        if(index == 0):
            matrix_length = re.sub('[^0-9]', '', line)
        else:
            new_line = re.sub('\n', '', line)
            dist = re.split(' ', new_line)
            matrix_dist.append(dist)

    file = open(sys.argv[2], 'r')
    text = file.readlines()
    for index, line in enumerate(text):
        if(index != 0):
            new_line = re.sub('\n', '', line)
            dist = re.split(' ', new_line)
            matrix_adj.append(dist)

    SearchOne = AStar(matrix_dist, matrix_adj, 0, 5)

    print('Final cost: ' + str(SearchOne.getFinalCost()))
    print('Route: ' + str(SearchOne.getRoute()))
    print('\n\n')

    SearchTwo = AStar(matrix_dist, matrix_adj, 1, 4)

    print('Final cost: ' + str(SearchTwo.getFinalCost()))
    print('Route: ' + str(SearchTwo.getRoute()))
