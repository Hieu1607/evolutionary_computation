"""
Integrated Path Planning System
Hệ thống quy hoạch đường đi tích hợp IACO + B-spline

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import Dict, Optional
import sys
import os

# Thêm đường dẫn
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environment.grid_map import GridMap
from src.aco.iaco import ImprovedACO
from src.ackermann.ackermann_model import AckermannModel
from src.path_smoothing.control_point_optimizer import BSplineSmoothing


class PathPlanningSystem:
    """
    Hệ thống quy hoạch đường đi tích hợp
    
    Bước 1: IACO tìm đường đi tối ưu
    Bước 2: B-spline làm mượt đường đi
    Bước 3: Kiểm tra ràng buộc Ackermann
    """
    
    # Tham số đề xuất (Bảng 1)
    DEFAULT_PARAMS = {
        'n_iterations': 100,      # K: Số lần lặp tối đa
        'n_ants': 50,            # M: Số kiến
        'alpha': 1.0,            # α: Hệ số pheromone
        'beta': 7.0,             # β: Hệ số kỳ vọng
        'rho': 0.3,              # ρ: Hệ số bay hơi
        'Q': 100.0,              # Q: Hằng số pheromone
        'lambda1': 0.6,          # λ₁: Trọng số khoảng cách
        'lambda2': 0.4,          # λ₂: Trọng số góc quay
        'P': 3.0,                # P: Hệ số phạt góc
        'k_L': 0.7,              # k_L: Trọng số độ dài
        'k_E': 0.3,              # k_E: Trọng số năng lượng
        'L': 2.5,                # L: Wheelbase robot (m)
        'phi_max_deg': 30.0,     # φ_max: Góc lái tối đa (độ)
    }
    
    def __init__(self,
                 grid_map: GridMap,
                 start: int,
                 goal: int,
                 params: Optional[Dict] = None):
        """
        Khởi tạo hệ thống quy hoạch đường đi
        
        Args:
            grid_map: Bản đồ lưới
            start: Node bắt đầu
            goal: Node đích
            params: Tham số tùy chỉnh (sử dụng DEFAULT_PARAMS nếu None)
        """
        self.grid_map = grid_map
        self.start = start
        self.goal = goal
        
        # Sử dụng tham số mặc định nếu không có tùy chỉnh
        self.params = self.DEFAULT_PARAMS.copy()
        if params is not None:
            self.params.update(params)
        
        # Khởi tạo mô hình Ackermann
        self.ackermann = AckermannModel(
            L=self.params['L'],
            phi_max_deg=self.params['phi_max_deg']
        )
        
        # Khởi tạo IACO
        self.iaco = ImprovedACO(
            grid_map=grid_map,
            start=start,
            goal=goal,
            n_ants=self.params['n_ants'],
            n_iterations=self.params['n_iterations'],
            alpha=self.params['alpha'],
            beta=self.params['beta'],
            rho=self.params['rho'],
            Q=self.params['Q'],
            lambda1=self.params['lambda1'],
            lambda2=self.params['lambda2'],
            P=self.params['P'],
            k_L=self.params['k_L'],
            k_E=self.params['k_E']
        )
        
        # Khởi tạo B-spline smoother
        self.smoother = BSplineSmoothing(
            ackermann=self.ackermann,
            grid_size=grid_map.a
        )
        
        # Kết quả
        self.raw_path = None
        self.smooth_path = None
        self.results = None
        
        print("\n" + "="*60)
        print("HỆ THỐNG QUY HOẠCH ĐƯỜNG ĐI TÍCH HỢP")
        print("="*60)
        print("IACO + B-spline + Ackermann Constraints")
        print("="*60 + "\n")
    
    def plan(self, verbose: bool = True) -> Dict:
        """
        Quy hoạch đường đi hoàn chỉnh
        
        Args:
            verbose: Hiển thị thông tin chi tiết
            
        Returns:
            Dictionary chứa kết quả
        """
        print("="*60)
        print("BƯỚC 1: TÌM ĐƯỜNG ĐI BẰNG IACO")
        print("="*60)
        
        # Bước 1: Tìm đường đi bằng IACO
        iaco_results = self.iaco.run(verbose=verbose)
        
        if not iaco_results['success']:
            print("\n✗ IACO không tìm thấy đường đi!")
            return {
                'success': False,
                'message': 'IACO failed',
                'iaco_results': iaco_results
            }
        
        self.raw_path = iaco_results['path_coords']
        
        print("\n✓ IACO hoàn thành!")
        print(f"  - Số nodes: {iaco_results['n_nodes']}")
        print(f"  - Độ dài: {iaco_results['length']:.4f}")
        print(f"  - Chi phí: {iaco_results['cost']:.4f}")
        
        # Bước 2: Làm mượt đường đi bằng B-spline
        print("\n" + "="*60)
        print("BƯỚC 2: LÀM MƯỢT ĐƯỜNG ĐI BẰNG B-SPLINE")
        print("="*60)
        
        smooth_results = self.smoother.smooth_path(
            self.raw_path,
            n_samples=100,
            optimize=True
        )
        
        self.smooth_path = smooth_results['smooth_path']
        
        print("\n✓ B-spline hoàn thành!")
        print(f"  - Số điểm mượt: {len(self.smooth_path)}")
        print(f"  - Độ cong max: {smooth_results['max_curvature']:.6f}")
        print(f"  - ρ_max cho phép: {self.ackermann.rho_max:.6f}")
        print(f"  - Ràng buộc Ackermann: {'Thỏa mãn ✓' if smooth_results['is_valid'] else 'Vi phạm ✗'}")
        
        # Tổng hợp kết quả
        self.results = {
            'success': True,
            'iaco': iaco_results,
            'smoothing': smooth_results,
            'raw_path': self.raw_path,
            'smooth_path': self.smooth_path,
            'ackermann_valid': smooth_results['is_valid']
        }
        
        return self.results
    
    def print_summary(self):
        """In tóm tắt kết quả"""
        if self.results is None:
            print("Chưa chạy quy hoạch!")
            return
        
        print("\n" + "="*60)
        print("TÓM TẮT KẾT QUẢ QUY HOẠCH ĐƯỜNG ĐI")
        print("="*60)
        
        # IACO results
        iaco = self.results['iaco']
        print("\n1. THUẬT TOÁN IACO:")
        print(f"   ✓ Tìm thấy đường đi")
        print(f"   - Số nodes:        {iaco['n_nodes']}")
        print(f"   - Độ dài L(p):     {iaco['length']:.4f}")
        print(f"   - Năng lượng E(p): {iaco['energy']:.4f}")
        print(f"   - Chi phí S(p):    {iaco['cost']:.4f}")
        
        # Smoothing results
        smooth = self.results['smoothing']
        print("\n2. LÀM MƯỢT B-SPLINE:")
        print(f"   ✓ Tạo đường mượt")
        print(f"   - Điểm kiểm soát:  {len(smooth['control_points'])}")
        print(f"   - Số điểm mượt:    {len(smooth['smooth_path'])}")
        print(f"   - Độ cong max:     {smooth['max_curvature']:.6f}")
        print(f"   - Độ cong mean:    {smooth['mean_curvature']:.6f}")
        
        # Ackermann constraints
        print("\n3. RÀNG BUỘC ACKERMANN:")
        print(f"   - Wheelbase L:     {self.ackermann.L:.2f} m")
        print(f"   - φ_max:           {np.degrees(self.ackermann.phi_max):.2f}°")
        print(f"   - ρ_max:           {self.ackermann.rho_max:.6f} m⁻¹")
        print(f"   - Trạng thái:      {'✓ Thỏa mãn' if smooth['is_valid'] else '✗ Vi phạm'}")
        
        # Parameters used
        print("\n4. THAM SỐ SỬ DỤNG:")
        print(f"   - Số kiến M:       {self.params['n_ants']}")
        print(f"   - Iterations K:    {self.params['n_iterations']}")
        print(f"   - α (pheromone):   {self.params['alpha']}")
        print(f"   - β (heuristic):   {self.params['beta']}")
        print(f"   - ρ (bay hơi):     {self.params['rho']}")
        print(f"   - λ₁, λ₂:          {self.params['lambda1']}, {self.params['lambda2']}")
        print(f"   - k_L, k_E:        {self.params['k_L']}, {self.params['k_E']}")
        
        print("\n" + "="*60)
        print("✓ QUY HOẠCH ĐƯỜNG ĐI HOÀN TẤT!")
        print("="*60 + "\n")


if __name__ == "__main__":
    # Test Integrated System
    print("Test Integrated Path Planning System\n")
    
    # Tạo grid map
    grid_map = GridMap(a=1.0, x_max=50.0, y_max=50.0)
    
    # Thêm chướng ngại vật
    obstacles = []
    
    # Chướng ngại vật ngang
    for i in range(15, 35):
        for j in range(20, 25):
            idx = grid_map.matrix_pos_to_grid_index(i, j)
            obstacles.append(idx)
    
    # Chướng ngại vật dọc
    for i in range(25, 40):
        for j in range(30, 35):
            idx = grid_map.matrix_pos_to_grid_index(i, j)
            obstacles.append(idx)
    
    grid_map.add_obstacles_batch(obstacles)
    
    # Điểm start và goal
    start = grid_map.coord_to_grid_index(2, 2)
    goal = grid_map.coord_to_grid_index(48, 48)
    
    print(f"Start: node {start}")
    print(f"Goal: node {goal}\n")
    
    # Tạo hệ thống với tham số đề xuất
    system = PathPlanningSystem(
        grid_map=grid_map,
        start=start,
        goal=goal,
        params=None  # Sử dụng tham số mặc định
    )
    
    # Chạy quy hoạch
    results = system.plan(verbose=True)
    
    # In tóm tắt
    system.print_summary()
