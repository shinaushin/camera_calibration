import numpy as np

class Map:
    def __init__(self, txt_file):
        self.ascii_map = []
        with open(txt_file) as f:
            row = 0
            for line in f:
                line_split = list(line)
                line_split = line_split[:len(line_split)-1]
                if len(line_split) != 0:
                    self.ascii_map.append(line_split)
                    for i in range(len(line_split)):
                        if line_split[i] == 'G':
                            self.goal_loc = [row, i]
                        if line_split[i] == 'R':
                            self.robot_start = [row, i]
                row = row + 1
        
        # print(self.map)
        # print(len(self.map), len(self.map[0]))
        # print(self.goal_loc)
        # print(self.robot_start)

        self.rows = len(self.ascii_map) # 65
        self.cols = len(self.ascii_map[0]) # 78
        self.num_nodes = self.rows * self.cols
        self.adj_matrix = np.ones((self.num_nodes, self.num_nodes)) - \
            np.identity(self.num_nodes)
        for i in range(self.num_nodes):
            for j in range(self.num_nodes):
                if i != j:
                    self.adj_matrix[i,j] = np.inf

        for i in range(self.num_nodes):
            row = int(i / self.cols)
            col = int(i % self.cols)
            if self.ascii_map[row][col] != '#':
                if col > 0 and self.ascii_map[row][col-1] != '#':
                    self.adj_matrix[i, i-1] = 1
                if col < self.cols - 1 and self.ascii_map[row][col+1] != '#':
                    self.adj_matrix[i,i+1] = 1
                if row > 0 and self.ascii_map[row-1][col] != '#':
                    self.adj_matrix[i,i-self.cols] = 1
                if row < self.rows - 1 and self.ascii_map[row+1][col] != '#':
                    self.adj_matrix[i,i+self.cols] = 1
        # print(adj_matrix[80,81])

    def djikstra(self):
        dist = np.ones((self.num_nodes, 1)) * np.inf
        prev = np.zeros((self.num_nodes, 1))
        dist[self.robot_start[0]*self.cols + self.robot_start[1]] = 0
        U = np.arange(self.num_nodes)
        goal_node = self.goal_loc[0]*self.cols + self.goal_loc[1]
        while goal_node in U:
            C = U[0]
            smallest_dist = dist[C]
            for i in range(2, len(U)):
                if dist[U[i]] < smallest_dist:
                    C = U[i]
                    smallest_dist = dist[U[i]]

            for i in range(len(U)):
                if U[i] == C:
                    U = np.delete(U, i)
                    break

            for i in range(self.num_nodes):
                if self.adj_matrix[C,i] != 0 and self.adj_matrix[C,i] != np.inf:
                    alt = dist[C] + self.adj_matrix[C,i]
                    if alt < dist[i]:
                        dist[i] = alt
                        prev[i] = C
        
        if dist[self.goal_loc[0]*self.cols + self.goal_loc[1]] == np.inf:
            self.display_path([])
            print("No path found.")
        else:
            path = [ self.goal_loc ]
            previous = prev[self.goal_loc[0]*self.cols + self.goal_loc[1]]
            while previous != self.robot_start[0]*self.cols + self.robot_start[1]:
                path.append([int(previous/self.cols), int(previous%self.cols)])
                previous = prev[int(previous)]
            path.reverse()
            self.display_path(path[:-1])

    def display_path(self, path):
        for loc in path:
            x = int(loc[0])
            y = int(loc[1])
            self.ascii_map[x][y] = 'O'
        result = open("map_with_path.txt", "w")
        for line in self.ascii_map:
            ascii_line = ''.join(line)
            result.write(ascii_line + "\n")
        result.close()


    @staticmethod
    def calc_dist(p1, p2):
        return np.sqrt((p1[0]-p2[0])**2  + (p1[1]-p2[1])**2)

    def obstacle_tracing(self):
        while self.robot_start != self.goal_loc:
            # 4 choices
            up = [self.robot_start[0]+1, self.robot_start[1]]
            down = [self.robot_start[0]-1, self.robot_start[1]]
            left = [self.robot_start[0], self.robot_start[1]-1]
            right = [self.robot_start[0], self.robot_start[1]+1]
            dist_up = self.calc_dist(up, self.goal_loc)
            dist_down = self.calc_dist(down, self.goal_loc)
            dist_left = self.calc_dist(left, self.goal_loc)
            dist_right = self.calc_dist(right, self.goal_loc)
            dists = [dist_up, dist_down, dist_left, dist_right]
            index = dists.index(min(dists))
            if index == 0:
                pass
            elif index == 1:
                pass
            elif index == 2:
                pass
            else:
                pass

    def A_star(self):
        pass


sol = Map("Programming Test A Data File.txt")
sol.djikstra()
# sol.obstacle_tracing()
# sol.A_star()
