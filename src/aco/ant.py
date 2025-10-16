"""
Ant Module
Mô-đun mô phỏng kiến trong thuật toán ACO

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import List, Tuple, Optional
import sys
import os

# Thêm đường dẫn để import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from environment.grid_map import GridMap
from aco.heuristic import HeuristicFunction


class Ant:
    """
    Lớp mô phỏng một kiến trong thuật toán ACO
    
    Attributes:
        grid_map: Bản đồ lưới
        heuristic: Hàm heuristic
        alpha: Hệ số pheromone
        beta: Hệ số kỳ vọng
        start: Node bắt đầu
        goal: Node đích
        path: Đường đi của kiến
        path_coords: Tọa độ của đường đi
    """
    
    def __init__(self, grid_map: GridMap, 
                 heuristic: HeuristicFunction,
                 alpha: float = 2.0,
                 beta: float = 5.0):
        """
        Khởi tạo Ant
        
        Args:
            grid_map: Bản đồ lưới
            heuristic: Hàm heuristic
            alpha: Hệ số pheromone α, thường trong [1, 4]
            beta: Hệ số kỳ vọng β, thường trong [4, 7]
        """
        assert 1 <= alpha <= 4, "α nên trong khoảng [1, 4]"
        assert 4 <= beta <= 7, "β nên trong khoảng [4, 7]"
        
        self.grid_map = grid_map
        self.heuristic = heuristic
        self.alpha = alpha
        self.beta = beta
        
        # Đường đi
        self.path = []
        self.path_coords = []
        self.visited = set()
        
        # Điểm bắt đầu và đích
        self.start = None
        self.goal = None
    
    def initialize(self, start: int, goal: int):
        """
        Khởi tạo kiến với điểm bắt đầu và đích
        
        Args:
            start: Node bắt đầu
            goal: Node đích
        """
        self.start = start
        self.goal = goal
        self.path = [start]
        self.visited = {start}
        
        # Lưu tọa độ
        x, y = self.grid_map.grid_index_to_coord(start)
        self.path_coords = [(x, y)]
    
    def calculate_transition_probability(self, current: int, 
                                         neighbors: List[int],
                                         pheromone_matrix: np.ndarray) -> np.ndarray:
        """
        Tính xác suất chuyển trạng thái
        
        Công thức (2):
        P^k_ij = [τ_ij(t)]^α × [η_ij(t)]^β / Σ[τ_ij(t)]^α × [η_ij(t)]^β
        
        Args:
            current: Node hiện tại
            neighbors: Danh sách các node có thể chọn (allowed_k)
            pheromone_matrix: Ma trận pheromone
            
        Returns:
            Mảng xác suất tương ứng với mỗi neighbor
        """
        if len(neighbors) == 0:
            return np.array([])
        
        # Lấy tọa độ đích
        x_goal, y_goal = self.grid_map.grid_index_to_coord(self.goal)
        
        # Lấy tọa độ hiện tại
        x_curr, y_curr = self.grid_map.grid_index_to_coord(current)
        
        # Lấy tọa độ trước đó (nếu có)
        if len(self.path) >= 2:
            x_prev, y_prev = self.grid_map.grid_index_to_coord(self.path[-2])
        else:
            x_prev, y_prev = x_curr, y_curr
        
        probabilities = []
        
        for neighbor in neighbors:
            # Lấy pheromone (chuyển từ 1-based sang 0-based indexing)
            tau_ij = pheromone_matrix[current - 1, neighbor - 1]
            
            # Lấy tọa độ neighbor
            x_next, y_next = self.grid_map.grid_index_to_coord(neighbor)
            
            # Tính heuristic
            if len(self.path) >= 2:
                # Có đủ thông tin để tính góc quay
                eta_ij = self.heuristic.calculate_heuristic_full(
                    x_prev, y_prev, x_curr, y_curr,
                    x_next, y_next, x_goal, y_goal
                )
            else:
                # Chưa có đủ thông tin, chỉ dùng khoảng cách
                eta_ij = self.heuristic.calculate_heuristic(
                    x_next, y_next, x_goal, y_goal, delta_theta=0.0
                )
            
            # Tính [τ_ij]^α × [η_ij]^β
            value = (tau_ij ** self.alpha) * (eta_ij ** self.beta)
            probabilities.append(value)
        
        # Chuẩn hóa thành xác suất
        probabilities = np.array(probabilities)
        total = np.sum(probabilities)
        
        if total > 0:
            probabilities = probabilities / total
        else:
            # Nếu tổng = 0, chọn ngẫu nhiên đều
            probabilities = np.ones(len(neighbors)) / len(neighbors)
        
        return probabilities
    
    def select_next_node(self, current: int, 
                        pheromone_matrix: np.ndarray) -> Optional[int]:
        """
        Chọn node tiếp theo dựa trên xác suất
        
        Args:
            current: Node hiện tại
            pheromone_matrix: Ma trận pheromone
            
        Returns:
            Node tiếp theo hoặc None nếu không thể di chuyển
        """
        # Lấy các node láng giềng chưa thăm
        all_neighbors = self.grid_map.get_neighbors(current, connectivity=8)
        allowed = [n for n in all_neighbors if n not in self.visited]
        
        if len(allowed) == 0:
            return None
        
        # Tính xác suất
        probabilities = self.calculate_transition_probability(
            current, allowed, pheromone_matrix
        )
        
        # Chọn node dựa trên xác suất
        next_node = np.random.choice(allowed, p=probabilities)
        
        return next_node
    
    def construct_solution(self, pheromone_matrix: np.ndarray,
                          max_steps: int = 10000) -> bool:
        """
        Xây dựng giải pháp (tìm đường đi)
        
        Args:
            pheromone_matrix: Ma trận pheromone
            max_steps: Số bước tối đa
            
        Returns:
            True nếu tìm thấy đường đi, False nếu không
        """
        current = self.start
        steps = 0
        
        while current != self.goal and steps < max_steps:
            # Chọn node tiếp theo
            next_node = self.select_next_node(current, pheromone_matrix)
            
            if next_node is None:
                # Không thể di chuyển tiếp, thất bại
                return False
            
            # Di chuyển đến node tiếp theo
            self.path.append(next_node)
            self.visited.add(next_node)
            
            # Lưu tọa độ
            x, y = self.grid_map.grid_index_to_coord(next_node)
            self.path_coords.append((x, y))
            
            current = next_node
            steps += 1
        
        # Kiểm tra đã đến đích chưa
        return current == self.goal
    
    def get_path(self) -> List[int]:
        """Lấy đường đi (danh sách node indices)"""
        return self.path.copy()
    
    def get_path_coords(self) -> List[Tuple[float, float]]:
        """Lấy tọa độ đường đi"""
        return self.path_coords.copy()
    
    def get_path_length(self) -> int:
        """Lấy số node trong đường đi"""
        return len(self.path)
    
    def reset(self):
        """Reset kiến về trạng thái ban đầu"""
        self.path = []
        self.path_coords = []
        self.visited = set()


if __name__ == "__main__":
    # Test Ant
    print("Test Ant Module\n")
    
    # Tạo grid map
    grid_map = GridMap(a=1.0, x_max=10.0, y_max=10.0)
    
    # Thêm vài chướng ngại vật
    grid_map.add_obstacles_batch([23, 24, 33, 34, 43, 44])
    
    # Tạo heuristic
    heuristic = HeuristicFunction(lambda1=0.6, lambda2=0.4, P=3.0)
    
    # Tạo kiến
    ant = Ant(grid_map, heuristic, alpha=2.0, beta=5.0)
    
    # Khởi tạo
    start = 1
    goal = 99
    ant.initialize(start, goal)
    
    print(f"Kiến bắt đầu từ node {start}, mục tiêu node {goal}")
    
    # Tạo pheromone matrix giả
    pheromone = np.ones((grid_map.total_grids, grid_map.total_grids))
    
    # Thử tìm đường
    print("\nĐang tìm đường...")
    success = ant.construct_solution(pheromone, max_steps=1000)
    
    if success:
        print(f"✓ Tìm thấy đường đi!")
        print(f"  - Số bước: {ant.get_path_length()}")
        print(f"  - Đường đi: {ant.get_path()[:10]}...")
    else:
        print("✗ Không tìm thấy đường đi")
