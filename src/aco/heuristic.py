"""
Heuristic Function Module
Mô-đun hàm heuristic cải tiến cho thuật toán kiến

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import Tuple, Optional


class HeuristicFunction:
    """
    Lớp tính toán hàm heuristic cải tiến cho thuật toán kiến
    
    Bao gồm:
    - Khoảng cách Euclidean đến đích
    - Góc quay của robot
    - Hệ số phạt góc lái
    - Hàm heuristic tổng hợp
    """
    
    def __init__(self, lambda1: float = 0.5, lambda2: float = 0.5, P: float = 3.0):
        """
        Khởi tạo Heuristic Function
        
        Args:
            lambda1: Trọng số khoảng cách (λ₁), mặc định 0.5
            lambda2: Trọng số góc quay (λ₂), mặc định 0.5
            P: Hệ số phạt góc lái, tối ưu trong [2, 4], mặc định 3.0
        """
        # Kiểm tra điều kiện λ₁ + λ₂ = 1
        assert abs(lambda1 + lambda2 - 1.0) < 1e-6, "λ₁ + λ₂ phải bằng 1"
        assert 0 <= lambda1 <= 1, "λ₁ phải trong khoảng [0, 1]"
        assert 0 <= lambda2 <= 1, "λ₂ phải trong khoảng [0, 1]"
        assert 2 <= P <= 4, "P nên trong khoảng [2, 4] để tối ưu"
        
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.P = P
        
        print(f"✓ Khởi tạo Heuristic Function")
        print(f"  - λ₁ (trọng số khoảng cách): {lambda1}")
        print(f"  - λ₂ (trọng số góc quay): {lambda2}")
        print(f"  - P (hệ số phạt): {P}")
    
    @staticmethod
    def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Tính khoảng cách Euclidean giữa hai điểm
        
        Công thức (11):
        d_jg = sqrt((x_j - x_g)² + (y_j - y_g)²)
        
        Args:
            x1, y1: Tọa độ điểm 1
            x2, y2: Tọa độ điểm 2
            
        Returns:
            Khoảng cách Euclidean
        """
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    @staticmethod
    def calculate_angle(x_prev: float, y_prev: float, 
                       x_curr: float, y_curr: float) -> float:
        """
        Tính góc của vector từ điểm trước đến điểm hiện tại
        
        Công thức (13-14):
        θ_i = arctan((y_i - y_{i-1})/(x_i - x_{i-1}))
        
        Args:
            x_prev, y_prev: Tọa độ điểm trước
            x_curr, y_curr: Tọa độ điểm hiện tại
            
        Returns:
            Góc θ theo radian, trong khoảng [-π, π]
        """
        return np.arctan2(y_curr - y_prev, x_curr - x_prev)
    
    def calculate_turn_angle(self, x_prev: float, y_prev: float,
                            x_curr: float, y_curr: float,
                            x_next: float, y_next: float) -> float:
        """
        Tính góc quay của robot
        
        Công thức (12):
        Δθ = |θ_i - θ_j|
        
        Args:
            x_prev, y_prev: Tọa độ điểm trước (i-1)
            x_curr, y_curr: Tọa độ điểm hiện tại (i)
            x_next, y_next: Tọa độ điểm tiếp theo (i+1)
            
        Returns:
            Góc quay Δθ theo radian, trong khoảng [0, π]
        """
        # Tính góc θ_i (từ i-1 đến i)
        theta_i = self.calculate_angle(x_prev, y_prev, x_curr, y_curr)
        
        # Tính góc θ_j (từ i đến i+1)
        theta_j = self.calculate_angle(x_curr, y_curr, x_next, y_next)
        
        # Tính góc quay Δθ
        delta_theta = abs(theta_j - theta_i)
        
        # Chuẩn hóa về [0, π]
        if delta_theta > np.pi:
            delta_theta = 2 * np.pi - delta_theta
        
        return delta_theta
    
    def steering_penalty(self, delta_theta: float) -> float:
        """
        Tính hệ số phạt góc lái
        
        Công thức (15):
        f_cost = {
            Δθ × P,           nếu Δθ ≤ π
            (2π - Δθ) × P,    nếu Δθ > π
        }
        
        Args:
            delta_theta: Góc quay Δθ
            
        Returns:
            Hệ số phạt f_cost
        """
        if delta_theta <= np.pi:
            f_cost = delta_theta * self.P
        else:
            f_cost = (2 * np.pi - delta_theta) * self.P
        
        return f_cost
    
    def calculate_heuristic(self, x_curr: float, y_curr: float,
                           x_goal: float, y_goal: float,
                           delta_theta: float = 0.0) -> float:
        """
        Tính hàm heuristic cải tiến η_ij
        
        Công thức (16):
        η_ij = 1 / (λ₁ × d_jg + λ₂ × f_cost)
        
        Args:
            x_curr, y_curr: Tọa độ điểm hiện tại
            x_goal, y_goal: Tọa độ điểm đích
            delta_theta: Góc quay (mặc định 0 nếu chưa có)
            
        Returns:
            Giá trị heuristic η_ij
        """
        # Tính khoảng cách đến đích (công thức 11)
        d_jg = self.euclidean_distance(x_curr, y_curr, x_goal, y_goal)
        
        # Tính hệ số phạt góc lái (công thức 15)
        f_cost = self.steering_penalty(delta_theta)
        
        # Tính heuristic (công thức 16)
        denominator = self.lambda1 * d_jg + self.lambda2 * f_cost
        
        # Tránh chia cho 0
        if denominator < 1e-10:
            return 1e10
        
        eta_ij = 1.0 / denominator
        
        return eta_ij
    
    def calculate_heuristic_full(self, x_prev: float, y_prev: float,
                                 x_curr: float, y_curr: float,
                                 x_next: float, y_next: float,
                                 x_goal: float, y_goal: float) -> float:
        """
        Tính heuristic đầy đủ với cả 3 điểm (prev, curr, next)
        
        Args:
            x_prev, y_prev: Tọa độ điểm trước
            x_curr, y_curr: Tọa độ điểm hiện tại
            x_next, y_next: Tọa độ điểm tiếp theo (candidate)
            x_goal, y_goal: Tọa độ điểm đích
            
        Returns:
            Giá trị heuristic η_ij
        """
        # Tính góc quay (công thức 12-14)
        delta_theta = self.calculate_turn_angle(x_prev, y_prev, 
                                                x_curr, y_curr,
                                                x_next, y_next)
        
        # Tính heuristic (công thức 16)
        return self.calculate_heuristic(x_next, y_next, x_goal, y_goal, delta_theta)
    
    def distance_to_goal(self, x: float, y: float, 
                        x_goal: float, y_goal: float) -> float:
        """
        Tính khoảng cách đến đích
        
        Args:
            x, y: Tọa độ hiện tại
            x_goal, y_goal: Tọa độ đích
            
        Returns:
            Khoảng cách Euclidean
        """
        return self.euclidean_distance(x, y, x_goal, y_goal)


class PathEvaluator:
    """
    Lớp đánh giá chất lượng đường đi
    
    Bao gồm:
    - Độ dài đường đi
    - Năng lượng tiêu tốn khi quay
    - Hàm mục tiêu tổng hợp
    """
    
    def __init__(self, k_L: float = 0.5, k_E: float = 0.5, P: float = 3.0):
        """
        Khởi tạo Path Evaluator
        
        Args:
            k_L: Trọng số độ dài đường đi
            k_E: Trọng số năng lượng tiêu tốn
            P: Hệ số phạt góc quay
        """
        # Kiểm tra điều kiện k_L + k_E = 1
        assert abs(k_L + k_E - 1.0) < 1e-6, "k_L + k_E phải bằng 1"
        assert 0 <= k_L <= 1, "k_L phải trong khoảng [0, 1]"
        assert 0 <= k_E <= 1, "k_E phải trong khoảng [0, 1]"
        
        self.k_L = k_L
        self.k_E = k_E
        self.P = P
        
        print(f"✓ Khởi tạo Path Evaluator")
        print(f"  - k_L (trọng số độ dài): {k_L}")
        print(f"  - k_E (trọng số năng lượng): {k_E}")
        print(f"  - P (hệ số phạt góc quay): {P}")
    
    def path_length(self, path_coords: np.ndarray) -> float:
        """
        Tính độ dài đường đi
        
        Công thức (17-18):
        L(p) = Σ(i=1 đến n-1) d(p_i, p_{i+1})
        d(p_i, p_{i+1}) = sqrt((x_{i+1} - x_i)² + (y_{i+1} - y_i)²)
        
        Args:
            path_coords: Ma trận Nx2 chứa tọa độ các điểm [(x1,y1), (x2,y2), ...]
            
        Returns:
            Độ dài tổng của đường đi
        """
        if len(path_coords) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path_coords) - 1):
            x_i, y_i = path_coords[i]
            x_next, y_next = path_coords[i + 1]
            
            # Tính khoảng cách giữa hai điểm liên tiếp
            distance = np.sqrt((x_next - x_i)**2 + (y_next - y_i)**2)
            length += distance
        
        return length
    
    def energy_consumption(self, path_coords: np.ndarray) -> float:
        """
        Tính năng lượng tiêu tốn khi quay
        
        Công thức (19-21):
        E(p) = Σ(i=2 đến n-1) P × |θ_{i+1} - θ_i|
        θ_{i+1} = arctan((y_{i+1} - y_i)/(x_{i+1} - x_i))
        θ_i = arctan((y_i - y_{i-1})/(x_i - x_{i-1}))
        
        Args:
            path_coords: Ma trận Nx2 chứa tọa độ các điểm
            
        Returns:
            Năng lượng tiêu tốn do góc quay
        """
        if len(path_coords) < 3:
            return 0.0
        
        energy = 0.0
        
        for i in range(1, len(path_coords) - 1):
            x_prev, y_prev = path_coords[i - 1]
            x_curr, y_curr = path_coords[i]
            x_next, y_next = path_coords[i + 1]
            
            # Tính góc θ_i (từ i-1 đến i)
            theta_i = np.arctan2(y_curr - y_prev, x_curr - x_prev)
            
            # Tính góc θ_{i+1} (từ i đến i+1)
            theta_next = np.arctan2(y_next - y_curr, x_next - x_curr)
            
            # Tính năng lượng tiêu tốn
            delta_theta = abs(theta_next - theta_i)
            energy += self.P * delta_theta
        
        return energy
    
    def evaluate_path(self, path_coords: np.ndarray) -> dict:
        """
        Đánh giá tổng hợp chất lượng đường đi
        
        Công thức (22-23):
        S(p) = k_L × L(p) + k_E × E(p)
        min S(p) = min(k_L × L(p) + k_E × E(p))
        
        Args:
            path_coords: Ma trận Nx2 chứa tọa độ các điểm
            
        Returns:
            Dictionary chứa:
            - length: Độ dài đường đi
            - energy: Năng lượng tiêu tốn
            - cost: Hàm mục tiêu tổng hợp S(p)
        """
        # Tính độ dài (công thức 17-18)
        length = self.path_length(path_coords)
        
        # Tính năng lượng (công thức 19-21)
        energy = self.energy_consumption(path_coords)
        
        # Tính hàm mục tiêu (công thức 22-23)
        cost = self.k_L * length + self.k_E * energy
        
        return {
            'length': length,
            'energy': energy,
            'cost': cost,
            'k_L': self.k_L,
            'k_E': self.k_E
        }
    
    def print_evaluation(self, path_coords: np.ndarray):
        """In kết quả đánh giá đường đi"""
        eval_result = self.evaluate_path(path_coords)
        
        print("\n" + "="*50)
        print("ĐÁNH GIÁ CHẤT LƯỢNG ĐƯỜNG ĐI")
        print("="*50)
        print(f"Số điểm:              {len(path_coords)}")
        print(f"Độ dài L(p):          {eval_result['length']:.4f}")
        print(f"Năng lượng E(p):      {eval_result['energy']:.4f}")
        print(f"Chi phí S(p):         {eval_result['cost']:.4f}")
        print(f"  (k_L={eval_result['k_L']}, k_E={eval_result['k_E']})")
        print("="*50 + "\n")


if __name__ == "__main__":
    # Test Heuristic Function
    print("Test Heuristic Function Module\n")
    
    # Khởi tạo heuristic
    heuristic = HeuristicFunction(lambda1=0.6, lambda2=0.4, P=3.0)
    
    # Test tính khoảng cách
    print("\nTest khoảng cách Euclidean:")
    d = heuristic.euclidean_distance(0, 0, 3, 4)
    print(f"  Khoảng cách từ (0,0) đến (3,4): {d:.2f}")
    
    # Test tính góc
    print("\nTest tính góc:")
    theta = heuristic.calculate_angle(0, 0, 1, 1)
    print(f"  Góc từ (0,0) đến (1,1): {np.degrees(theta):.2f}°")
    
    # Test tính góc quay
    print("\nTest góc quay:")
    delta = heuristic.calculate_turn_angle(0, 0, 1, 0, 1, 1)
    print(f"  Góc quay: {np.degrees(delta):.2f}°")
    
    # Test heuristic
    print("\nTest heuristic:")
    eta = heuristic.calculate_heuristic_full(0, 0, 1, 0, 2, 1, 10, 10)
    print(f"  Giá trị heuristic: {eta:.6f}")
    
    # Test Path Evaluator
    print("\n" + "="*50)
    print("Test Path Evaluator")
    print("="*50)
    
    evaluator = PathEvaluator(k_L=0.7, k_E=0.3, P=3.0)
    
    # Tạo đường đi test
    path = np.array([
        [0, 0],
        [1, 0],
        [2, 1],
        [3, 2],
        [4, 2]
    ])
    
    evaluator.print_evaluation(path)
