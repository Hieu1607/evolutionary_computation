"""
Improved Ant Colony Optimization (IACO-TAC) Module
Thuật toán kiến cải tiến với cơ chế hai tầng

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import sys
import os

# Thêm đường dẫn để import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from environment.grid_map import GridMap
from aco.heuristic import HeuristicFunction, PathEvaluator
from aco.pheromone import PheromoneMatrix
from aco.ant import Ant


class ImprovedACO:
    """
    Thuật toán kiến cải tiến (IACO-TAC)
    
    Tính năng:
    - Hàm heuristic cải tiến với góc quay
    - Cập nhật pheromone hai tầng
    - Cơ chế thưởng phạt
    - Giới hạn pheromone MMAS
    - Đánh giá đa mục tiêu
    """
    
    def __init__(self,
                 grid_map: GridMap,
                 start: int,
                 goal: int,
                 n_ants: int = 30,
                 n_iterations: int = 100,
                 alpha: float = 2.0,
                 beta: float = 5.0,
                 rho: float = 0.3,
                 Q: float = 100.0,
                 lambda1: float = 0.6,
                 lambda2: float = 0.4,
                 P: float = 3.0,
                 k_L: float = 0.7,
                 k_E: float = 0.3,
                 tau_min: float = 0.01,
                 tau_max: float = 10.0):
        """
        Khởi tạo IACO
        
        Args:
            grid_map: Bản đồ lưới
            start: Node bắt đầu
            goal: Node đích
            n_ants: Số lượng kiến
            n_iterations: Số lần lặp
            alpha: Hệ số pheromone α ∈ [1, 4]
            beta: Hệ số kỳ vọng β ∈ [4, 7]
            rho: Hệ số bay hơi ρ ∈ [0.2, 0.5]
            Q: Hằng số pheromone
            lambda1: Trọng số khoảng cách (λ₁)
            lambda2: Trọng số góc quay (λ₂)
            P: Hệ số phạt góc lái ∈ [2, 4]
            k_L: Trọng số độ dài
            k_E: Trọng số năng lượng
            tau_min: Pheromone tối thiểu
            tau_max: Pheromone tối đa
        """
        self.grid_map = grid_map
        self.start = start
        self.goal = goal
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        
        # Tham số thuật toán
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.Q = Q
        
        # Khởi tạo các thành phần
        self.heuristic = HeuristicFunction(lambda1, lambda2, P)
        self.evaluator = PathEvaluator(k_L, k_E, P)
        self.pheromone = PheromoneMatrix(
            grid_map.total_grids, rho, Q, tau_min, tau_max
        )
        
        # Lưu trữ kết quả
        self.best_path = None
        self.best_path_coords = None
        self.best_cost = float('inf')
        self.history = {
            'best_costs': [],
            'mean_costs': [],
            'worst_costs': [],
            'iteration': []
        }
        
        print("\n" + "="*60)
        print("KHỞI TẠO THUẬT TOÁN KIẾN CẢI TIẾN (IACO-TAC)")
        print("="*60)
        print(f"Số kiến:          {n_ants}")
        print(f"Số iterations:    {n_iterations}")
        print(f"α (pheromone):    {alpha}")
        print(f"β (heuristic):    {beta}")
        print(f"ρ (bay hơi):      {rho}")
        print(f"Q (hằng số):      {Q}")
        print(f"λ₁, λ₂:           {lambda1}, {lambda2}")
        print(f"P (phạt góc):     {P}")
        print(f"k_L, k_E:         {k_L}, {k_E}")
        print("="*60 + "\n")
    
    def run(self, verbose: bool = True) -> Dict:
        """
        Chạy thuật toán IACO
        
        Args:
            verbose: Hiển thị thông tin chi tiết
            
        Returns:
            Dictionary chứa kết quả
        """
        print("BẮT ĐẦU TÌM KIẾM ĐƯỜNG ĐI...")
        print("-" * 60)
        
        for iteration in range(self.n_iterations):
            # Danh sách lưu đường đi và chi phí của các kiến
            ant_paths = []
            ant_path_coords = []
            ant_costs = []
            
            # Tạo và chạy các kiến
            for ant_id in range(self.n_ants):
                ant = Ant(self.grid_map, self.heuristic, self.alpha, self.beta)
                ant.initialize(self.start, self.goal)
                
                # Tìm đường
                success = ant.construct_solution(self.pheromone.pheromone)
                
                if success:
                    path = ant.get_path()
                    path_coords = np.array(ant.get_path_coords())
                    
                    # Đánh giá đường đi
                    eval_result = self.evaluator.evaluate_path(path_coords)
                    cost = eval_result['cost']
                    
                    ant_paths.append(path)
                    ant_path_coords.append(path_coords)
                    ant_costs.append(cost)
                    
                    # Cập nhật best solution
                    if cost < self.best_cost:
                        self.best_cost = cost
                        self.best_path = path
                        self.best_path_coords = path_coords
            
            # Nếu tìm thấy ít nhất một đường đi
            if len(ant_costs) > 0:
                # Cập nhật pheromone hai tầng
                self.pheromone.update_pheromone_two_layer(
                    ant_paths, ant_costs, self.best_cost
                )
                
                # Lưu lịch sử
                self.history['iteration'].append(iteration)
                self.history['best_costs'].append(min(ant_costs))
                self.history['mean_costs'].append(np.mean(ant_costs))
                self.history['worst_costs'].append(max(ant_costs))
                
                # Hiển thị thông tin
                if verbose and (iteration % 10 == 0 or iteration == self.n_iterations - 1):
                    print(f"Iter {iteration:3d}: "
                          f"Best={min(ant_costs):8.2f}, "
                          f"Mean={np.mean(ant_costs):8.2f}, "
                          f"Worst={max(ant_costs):8.2f}, "
                          f"Global Best={self.best_cost:8.2f}, "
                          f"Success={len(ant_costs)}/{self.n_ants}")
            else:
                if verbose:
                    print(f"Iter {iteration:3d}: Không tìm thấy đường đi nào!")
        
        print("-" * 60)
        print("HOÀN THÀNH!\n")
        
        # Trả về kết quả
        return self.get_results()
    
    def get_results(self) -> Dict:
        """
        Lấy kết quả thuật toán
        
        Returns:
            Dictionary chứa kết quả
        """
        if self.best_path is None:
            return {
                'success': False,
                'message': 'Không tìm thấy đường đi'
            }
        
        # Đánh giá đường đi tốt nhất
        eval_result = self.evaluator.evaluate_path(self.best_path_coords)
        
        return {
            'success': True,
            'path': self.best_path,
            'path_coords': self.best_path_coords,
            'cost': self.best_cost,
            'length': eval_result['length'],
            'energy': eval_result['energy'],
            'n_nodes': len(self.best_path),
            'history': self.history
        }
    
    def print_results(self):
        """In kết quả chi tiết"""
        results = self.get_results()
        
        if not results['success']:
            print(f"✗ {results['message']}")
            return
        
        print("\n" + "="*60)
        print("KẾT QUẢ THUẬT TOÁN IACO-TAC")
        print("="*60)
        print(f"✓ Tìm thấy đường đi thành công!")
        print(f"\nĐịnh lượng:")
        print(f"  Số nodes:          {results['n_nodes']}")
        print(f"  Độ dài L(p):       {results['length']:.4f}")
        print(f"  Năng lượng E(p):   {results['energy']:.4f}")
        print(f"  Chi phí S(p):      {results['cost']:.4f}")
        print(f"\nTham số đánh giá:")
        print(f"  k_L = {self.evaluator.k_L}")
        print(f"  k_E = {self.evaluator.k_E}")
        print(f"\nĐường đi (10 điểm đầu):")
        path_preview = results['path'][:10]
        print(f"  {path_preview}...")
        print("="*60 + "\n")


if __name__ == "__main__":
    # Test IACO
    print("Test IACO Module\n")
    
    # Tạo grid map
    grid_map = GridMap(a=1.0, x_max=20.0, y_max=20.0)
    
    # Thêm chướng ngại vật
    obstacles = []
    for i in range(5, 15):
        for j in range(8, 12):
            idx = grid_map.matrix_pos_to_grid_index(i, j)
            obstacles.append(idx)
    
    grid_map.add_obstacles_batch(obstacles)
    
    # Điểm bắt đầu và đích
    start = grid_map.coord_to_grid_index(1, 1)
    goal = grid_map.coord_to_grid_index(18, 18)
    
    print(f"Start: node {start}, Goal: node {goal}\n")
    
    # Khởi tạo IACO
    iaco = ImprovedACO(
        grid_map=grid_map,
        start=start,
        goal=goal,
        n_ants=20,
        n_iterations=50,
        alpha=2.0,
        beta=5.0,
        rho=0.3,
        lambda1=0.6,
        lambda2=0.4,
        k_L=0.7,
        k_E=0.3
    )
    
    # Chạy thuật toán
    results = iaco.run(verbose=True)
    
    # In kết quả
    iaco.print_results()
