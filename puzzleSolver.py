import sys
import argparse
import math
from itertools import *
from heapq import heappush, heappop


class Stack:

    def __init__(self):
        self.list = []

    def push(self, key):
        self.list.append(key)

    def pop(self):
        return self.list.pop()

    def is_empty(self):
        return len(self.list) == 0


class Astar:

    def __init__(self, start_state, heuristic, type):
        self.start_state = start_state
        if (heuristic == 1):
            self.heuristic = self.heuristic_manhattan
        elif (heuristic == 2):
            self.heuristic = self.heuristic_displaced
        self.type = type
        self.set_goal_state()
        self.explored = {}
        self.frontier = []
        self.IDAsol = None
        self.counter = 0

    def set_goal_state(self):
        if (self.type == 3):
            self.goal_state = ((1, 2, 3), (4, 5, 6), (7, 8, 0))
        elif (self.type == 4):
            self.goal_state = ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12),
                               (13, 14, 15, 0))

    def check_goal(self, curr_state):
        if (self.goal_state == curr_state):
            return True
        return False

    def possible_actions(self, curr_state):
        flag = 0
        i = 0
        j = 0
        for i, row in enumerate(curr_state):
            for j, num in enumerate(row):
                if num == 0:
                    flag = 1
                    break
            if flag == 1:
                break
        action_list = []
        if i != 0:
            action_list.append('U')
        if i != self.type - 1:
            action_list.append('D')
        if j != 0:
            action_list.append('L')
        if j != self.type - 1:
            action_list.append('R')
        return action_list

    def opposite(self, action):
        opp_action = ''
        if action == 'L':
            opp_action = 'R'
        elif action == 'R':
            opp_action = 'L'
        elif action == 'D':
            opp_action = 'U'
        elif action == 'U':
            opp_action = 'D'
        return opp_action

    def create_state(self, curr_state, action):
        flag = 0
        new_state = curr_state
        i = 0
        j = 0
        for i, row in enumerate(curr_state):
            for j, num in enumerate(row):
                if num == 0:
                    flag = 1
                    break
            if flag == 1:
                break
        new_state = list(map(list, new_state))
        if action == 'L':
            temp = new_state[i][j]
            new_state[i][j] = new_state[i][j - 1]
            new_state[i][j - 1] = temp
        elif action == 'R':
            temp = new_state[i][j]
            new_state[i][j] = new_state[i][j + 1]
            new_state[i][j + 1] = temp
        elif action == 'U':
            temp = new_state[i][j]
            new_state[i][j] = new_state[i - 1][j]
            new_state[i - 1][j] = temp
        elif action == 'D':
            temp = new_state[i][j]
            new_state[i][j] = new_state[i + 1][j]
            new_state[i + 1][j] = temp
        return tuple(tuple(row) for row in new_state)

    def A_star_solution(self):
        second_priority = 0
        self.start_node = self.create_node(parent_state=None, action=None)
        heappush(self.frontier,
                 (self.start_node['Fn'], second_priority - 1, self.start_node))
        second_priority += -1
        while self.frontier is not None:
            priority, _, curr_node = heappop(self.frontier)
            curr_state = curr_node['Current']
            if self.check_goal(curr_state) is False:
                if curr_state in self.explored:
                    continue
                else:
                    self.explored[curr_state] = curr_node
                    for action in self.possible_actions(curr_state):
                        if (action not in self.opposite(curr_node['Action'])):
                            new_node = self.create_node(curr_state, action)
                            heappush(
                                self.frontier,
                                (new_node['Fn'], second_priority - 1, new_node))
                            second_priority += -1
            else:
                break

        solution_temp = Stack()
        while curr_node['Parent'] is not None:
            solution_temp.push(curr_node['Action'])
            curr_node = self.explored[curr_node['Parent']]

        self.solution = ''
        while solution_temp.is_empty() is not True:
            self.solution = self.solution + solution_temp.pop() + ','
        self.solution = self.solution.rstrip(',')
        return self.solution

    def create_node(self, parent_state, action):
        node = {}
        node['Gn'] = 0
        if parent_state is None:
            new_state = self.start_state
        else:
            new_state = self.create_state(parent_state, action)
            node['Gn'] = self.explored[parent_state]['Gn'] + 1
        node['Hn'] = self.heuristic(new_state)
        node['Fn'] = node['Hn'] + node['Gn']
        node['Current'] = new_state
        node['Parent'] = parent_state
        node['Action'] = action
        return node

    def heuristic_manhattan(self, state):
        heu = 0
        for i, row in enumerate(state):
            for j, num in enumerate(row):
                if (num != 0):
                    heu += abs(int((num - 1) / self.type) - i)
                    heu += abs(int((num - 1) % self.type) - j)
        return heu

    def heuristic_displaced(self, state):
        heu = 0
        for i, row in enumerate(state):
            for j, num in enumerate(row):
                if (num == 0 and (i != self.type - 1 or j != self.type - 1)):
                    heu += 1
                else:
                    if (num == 0):
                        continue
                    elif (int((num - 1) / self.type) != i or int(
                        (num - 1) % self.type) != j):
                        heu += 1
        return heu


class IDAstar(Astar):

    def IDAstar_solution(self):
        initial_threshold = self.heuristic(self.start_state)
        while self.IDAsol is None:
            (self.IDAsol, updated_threshold) = self.IDAhelper(
                self.start_state, None, initial_threshold, 0)
            initial_threshold = updated_threshold
        return self.IDAsol.lstrip(',')

    def IDAhelper(self, state, action_used, threshold, Gn):
        self.counter += 1
        if Gn + self.heuristic(state) <= threshold:
            if self.check_goal(state) is False:
                suggested_threshold = 1000
                for action in self.possible_actions(state):
                    if action not in self.opposite(action_used):
                        new_state = self.create_state(state, action)
                        (solution, new_threshold) = self.IDAhelper(
                            new_state, action, threshold, Gn + 1)
                        if solution is None:
                            if suggested_threshold > new_threshold:
                                suggested_threshold = new_threshold
                        else:
                            return (',' + action + solution, threshold)
                return (None, suggested_threshold)
            else:
                return ('', Gn + self.heuristic(state))
        else:
            return (None, Gn + self.heuristic(state))


def input_parse(input_file):
    input = open(input_file).read()
    data = []
    for line in input.rstrip('\n').split('\n'):
        numbers = line.split(',')
        numberstemp = []
        for num in numbers:
            if num:
                numberstemp.append(int(num))
            else:
                numberstemp.append(0)
        data.append(numberstemp)
    print('Input:')
    print(data)
    return tuple(tuple(row) for row in data)


if __name__ == '__main__':
    algorithm = ''
    heuristic = 1
    if (str(sys.argv[1]) == '1'):
        algorithm = 1
        print("A*")
    elif (str(sys.argv[1]) == '2'):
        algorithm = 2
        print("IDA*")
    if (str(sys.argv[2]) == '3'):
        type = 3
    elif (str(sys.argv[2]) == '4'):
        type = 4
    if (str(sys.argv[3]) == '1'):
        heuristic = 1
        print("Manhatten Heuristic")
    elif (str(sys.argv[3]) == '2'):
        heuristic = 2
        print("Displaced Tiles Heuristic")
    input_file = str(sys.argv[4])
    output_file = str(sys.argv[5])
    start_state = input_parse(input_file)

    print('Output:')
    if (algorithm == 1):
        initialize = Astar(start_state, heuristic, type)
        solution = initialize.A_star_solution()
        print(solution)

    elif (algorithm == 2):
        initialize = IDAstar(start_state, heuristic, type)
        solution = initialize.IDAstar_solution()
        print(solution)

    output = open(output_file, "w")
    output.write(solution)
    output.close()
