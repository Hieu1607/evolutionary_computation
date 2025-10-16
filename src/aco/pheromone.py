"""
Pheromone Matrix Module
Mô-đun quản lý ma trận pheromone với cập nhật hai tầng

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import List, Tuple, Optional


class PheromoneMatrix:
    """
    Lớp quản lý ma trận pheromone cho thuật toán kiến
    
    Bao gồm:
    - Cập nhật pheromone hai tầng
    - Cơ chế thưởng phạt
    - Giới hạn nồng độ pheromone (MMAS)
    """
    
    def __init__(self, n_nodes: int, 
                 rho: float = 0.3,
                 Q: float = 100.0,
                 tau_min: float = 0.01,
                 tau_max: float = 10.0,
                 initial_pheromone: float = 1.0):
        """
        Khởi tạo Pheromone Matrix
        
        Args:
            n_nodes: Số lượng nodes (grids)
            rho: Hệ số bay hơi pheromone ρ ∈ (0,1), thường [0.2, 0.5]
            Q: Hằng số pheromone
            tau_min: Nồng độ pheromone tối thiểu (MMAS)
            tau_max: Nồng độ pheromone tối đa (MMAS)
            initial_pheromone: Nồng độ pheromone ban đầu
        """
        assert 0 < rho < 1, "ρ phải trong khoảng (0, 1)"
        assert 0.2 <= rho <= 0.5, "ρ nên trong khoảng [0.2, 0.5] để tối ưu"
        assert tau_min < tau_max, "τ_min phải nhỏ hơn τ_max"
        
        self.n_nodes = n_nodes
        self.rho = rho
        self.Q = Q
        self.tau_min = tau_min
        self.tau_max = tau_max
        
        # Khởi tạo ma trận pheromone
        self.pheromone = np.ones((n_nodes, n_nodes)) * initial_pheromone
        
        # Lưu trữ lịch sử pheromone
        self.pheromone_history = []
        
        print(f"✓ Khởi tạo Pheromone Matrix")
        print(f"  - Số nodes: {n_nodes}")
        print(f"  - ρ (bay hơi): {rho}")
        print(f"  - Q (hằng số): {Q}")
        print(f"  - τ_min: {tau_min}, τ_max: {tau_max}")
        print(f"  - Pheromone ban đầu: {initial_pheromone}")
    
    def evaporate(self):
        """
        Bay hơi pheromone
        Công thức: τ_ij(t) = (1 - ρ) × τ_ij(t)
        """
        self.pheromone *= (1 - self.rho)
    
    def calculate_reward_penalty(self, S_k: float, S_best: float, S_worst: float) -> int:
        """
        Tính hệ số thưởng phạt
        
        Công thức (27):
        w = {
            1,   nếu S ≤ (S_best + S_worst)/2
            -1,  nếu S > (S_best + S_worst)/2
        }
        
        Args:
            S_k: Chi phí của đường đi k
            S_best: Chi phí tốt nhất hiện tại
            S_worst: Chi phí xấu nhất hiện tại
            
        Returns:
            1 (thưởng) hoặc -1 (phạt)
        """
        threshold = (S_best + S_worst) / 2
        
        if S_k <= threshold:
            return 1  # Thưởng
        else:
            return -1  # Phạt
    
    def update_pheromone_from_ant(self, path: List[int], S_k: float,
                                   S_best: float, S_worst: float) -> np.ndarray:
        """
        Tính pheromone tăng thêm từ một kiến (với cơ chế thưởng phạt)
        
        Công thức (26):
        τ^k_ij(t) = {
            w × Q/S_k(t),  nếu kiến k đi qua đường (i,j)
            0,             ngược lại
        }
        
        Args:
            path: Đường đi của kiến (list các node indices)
            S_k: Chi phí của đường đi
            S_best: Chi phí tốt nhất hiện tại
            S_worst: Chi phí xấu nhất hiện tại
            
        Returns:
            Ma trận delta pheromone từ kiến này
        """
        delta_tau = np.zeros((self.n_nodes, self.n_nodes))
        
        # Tính hệ số thưởng phạt (công thức 27)
        w = self.calculate_reward_penalty(S_k, S_best, S_worst)
        
        # Tính lượng pheromone tăng thêm
        if S_k > 0:
            pheromone_amount = w * self.Q / S_k
            
            # Cập nhật pheromone cho các cạnh trên đường đi
            for i in range(len(path) - 1):
                node_i = path[i]
                node_j = path[i + 1]
                # Chuyển từ 1-based sang 0-based indexing
                delta_tau[node_i - 1, node_j - 1] += pheromone_amount
                delta_tau[node_j - 1, node_i - 1] += pheromone_amount  # Ma trận đối xứng
        
        return delta_tau
    
    def update_pheromone_two_layer(self, ant_paths: List[List[int]],
                                    ant_costs: List[float],
                                    S_Best: float):
        """
        Cập nhật pheromone hai tầng
        
        Công thức (24-25):
        τ_ij(t+1) = (1 - ρ) × τ_ij(t) + Δτ_ij(t) + Q/S_Best
        Δτ_ij(t) = Σ(k=1 đến m) τ^k_ij(t)
        
        Args:
            ant_paths: Danh sách đường đi của các kiến
            ant_costs: Danh sách chi phí tương ứng
            S_Best: Chi phí tốt nhất toàn cục
        """
        # Bước 1: Bay hơi pheromone
        self.evaporate()
        
        # Bước 2: Tính S_best và S_worst trong iteration này
        if len(ant_costs) > 0:
            S_best_iter = min(ant_costs)
            S_worst_iter = max(ant_costs)
        else:
            return
        
        # Bước 3: Tổng hợp pheromone từ tất cả kiến (công thức 25)
        delta_tau_total = np.zeros((self.n_nodes, self.n_nodes))
        
        for path, cost in zip(ant_paths, ant_costs):
            if len(path) > 1:
                delta_tau_k = self.update_pheromone_from_ant(
                    path, cost, S_best_iter, S_worst_iter
                )
                delta_tau_total += delta_tau_k
        
        # Bước 4: Thêm pheromone từ giải pháp tốt nhất toàn cục
        if S_Best > 0:
            elite_bonus = self.Q / S_Best
        else:
            elite_bonus = 0
        
        # Bước 5: Cập nhật pheromone (công thức 24)
        self.pheromone += delta_tau_total + elite_bonus
        
        # Bước 6: Áp dụng giới hạn MMAS (công thức 28)
        self.apply_mmas_limits()
    
    def apply_mmas_limits(self):
        """
        Áp dụng giới hạn nồng độ pheromone (MMAS)
        
        Công thức (28):
        τ = {
            τ_min,  nếu τ ≤ τ_min
            τ,      nếu τ_min < τ < τ_max
            τ_max,  nếu τ ≥ τ_max
        }
        """
        self.pheromone = np.clip(self.pheromone, self.tau_min, self.tau_max)
    
    def get_pheromone(self, i: int, j: int) -> float:
        """
        Lấy nồng độ pheromone giữa node i và j
        
        Args:
            i: Node thứ i (1-based index)
            j: Node thứ j (1-based index)
            
        Returns:
            Nồng độ pheromone τ_ij
        """
        # Chuyển từ 1-based sang 0-based indexing
        return self.pheromone[i - 1, j - 1]
    
    def reset_pheromone(self, initial_value: float = 1.0):
        """
        Đặt lại pheromone về giá trị ban đầu
        
        Args:
            initial_value: Giá trị pheromone ban đầu
        """
        self.pheromone = np.ones((self.n_nodes, self.n_nodes)) * initial_value
        print(f"✓ Đã reset pheromone về {initial_value}")
    
    def save_history(self):
        """Lưu trạng thái pheromone hiện tại vào lịch sử"""
        self.pheromone_history.append(self.pheromone.copy())
    
    def get_statistics(self) -> dict:
        """
        Lấy thống kê về pheromone
        
        Returns:
            Dictionary chứa thống kê
        """
        return {
            'mean': np.mean(self.pheromone),
            'std': np.std(self.pheromone),
            'min': np.min(self.pheromone),
            'max': np.max(self.pheromone),
            'median': np.median(self.pheromone)
        }
    
    def print_statistics(self):
        """In thống kê pheromone"""
        stats = self.get_statistics()
        
        print("\n" + "="*50)
        print("THỐNG KÊ PHEROMONE")
        print("="*50)
        print(f"Trung bình:  {stats['mean']:.4f}")
        print(f"Độ lệch:     {stats['std']:.4f}")
        print(f"Min:         {stats['min']:.4f}")
        print(f"Max:         {stats['max']:.4f}")
        print(f"Median:      {stats['median']:.4f}")
        print("="*50 + "\n")


if __name__ == "__main__":
    # Test Pheromone Matrix
    print("Test Pheromone Matrix Module\n")
    
    # Khởi tạo
    pheromone_matrix = PheromoneMatrix(
        n_nodes=100,
        rho=0.3,
        Q=100.0,
        tau_min=0.01,
        tau_max=10.0
    )
    
    # Test cơ chế thưởng phạt
    print("\nTest cơ chế thưởng phạt:")
    w1 = pheromone_matrix.calculate_reward_penalty(S_k=50, S_best=40, S_worst=80)
    w2 = pheromone_matrix.calculate_reward_penalty(S_k=70, S_best=40, S_worst=80)
    print(f"  S_k=50, S_best=40, S_worst=80 → w={w1} ({'Thưởng' if w1 > 0 else 'Phạt'})")
    print(f"  S_k=70, S_best=40, S_worst=80 → w={w2} ({'Thưởng' if w2 > 0 else 'Phạt'})")
    
    # Test cập nhật pheromone
    print("\nTest cập nhật pheromone:")
    ant_paths = [
        [0, 1, 2, 3, 4],
        [0, 5, 10, 15, 20],
        [0, 2, 4, 6, 8]
    ]
    ant_costs = [100.0, 120.0, 90.0]
    S_Best = 85.0
    
    pheromone_matrix.update_pheromone_two_layer(ant_paths, ant_costs, S_Best)
    pheromone_matrix.print_statistics()
    
    # Test MMAS limits
    print("Test MMAS limits:")
    print(f"  Pheromone[0,1] = {pheromone_matrix.get_pheromone(0, 1):.4f}")
    print(f"  (Nằm trong [{pheromone_matrix.tau_min}, {pheromone_matrix.tau_max}])")
