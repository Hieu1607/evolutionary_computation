"""
Control Point Optimizer Module
Tối ưu hóa điểm kiểm soát cho B-spline

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import List, Tuple, Optional
import sys
import os

# Thêm đường dẫn
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ackermann.ackermann_model import AckermannModel
from path_smoothing.bspline import BSpline


class ControlPointOptimizer:
    """
    Lớp tối ưu hóa điểm kiểm soát cho B-spline
    
    Chiến lược:
    - Thêm điểm kiểm soát dựa trên góc quay
    - Xử lý vi phạm ràng buộc độ cong
    """
    
    def __init__(self, grid_size: float = 1.0):
        """
        Khởi tạo Control Point Optimizer
        
        Args:
            grid_size: Kích thước grid
        """
        self.grid_size = grid_size
        
        print(f"✓ Khởi tạo Control Point Optimizer")
        print(f"  - Grid size: {grid_size}")
    
    @staticmethod
    def calculate_turn_angle(p_prev: np.ndarray, p_curr: np.ndarray,
                            p_next: np.ndarray) -> float:
        """
        Tính góc quay tại điểm p_curr
        
        Args:
            p_prev: Điểm trước
            p_curr: Điểm hiện tại
            p_next: Điểm tiếp theo
            
        Returns:
            Góc quay theo radian [0, π]
        """
        # Vector từ prev đến curr
        v1 = p_curr - p_prev
        # Vector từ curr đến next
        v2 = p_next - p_curr
        
        # Tính góc giữa hai vector
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        angle = np.arccos(cos_angle)
        
        return angle
    
    def add_control_point_by_angle(self, p_prev: np.ndarray,
                                   p_curr: np.ndarray,
                                   p_next: np.ndarray) -> List[np.ndarray]:
        """
        Thêm điểm kiểm soát dựa trên góc quay
        
        Quy tắc:
        - Nếu góc quay > 90°: Thêm điểm tại 1/3 khoảng cách từ điểm quay
        - Nếu góc quay ≤ 90°: Thêm điểm tại 1/5 khoảng cách từ điểm quay
        
        Args:
            p_prev: Điểm trước
            p_curr: Điểm hiện tại (điểm quay)
            p_next: Điểm tiếp theo
            
        Returns:
            Danh sách điểm mới (có thể có điểm bổ sung)
        """
        # Tính góc quay
        angle = self.calculate_turn_angle(p_prev, p_curr, p_next)
        angle_deg = np.degrees(angle)
        
        # Quyết định tỷ lệ dựa trên góc
        if angle_deg > 90:
            ratio = 1.0 / 3.0
        else:
            ratio = 1.0 / 5.0
        
        # Thêm điểm phụ trước điểm quay
        new_point_before = p_prev + ratio * (p_curr - p_prev)
        
        # Thêm điểm phụ sau điểm quay
        new_point_after = p_curr + ratio * (p_next - p_curr)
        
        return [new_point_before, p_curr, new_point_after]
    
    def insert_intermediate_points(self, path: np.ndarray,
                                   angle_threshold_deg: float = 45.0) -> np.ndarray:
        """
        Chèn điểm trung gian dựa trên góc quay
        
        Args:
            path: Đường đi ban đầu (Nx2)
            angle_threshold_deg: Ngưỡng góc để thêm điểm (độ)
            
        Returns:
            Đường đi mới với các điểm bổ sung
        """
        if len(path) < 3:
            return path
        
        new_path = [path[0]]  # Giữ điểm đầu
        
        for i in range(1, len(path) - 1):
            p_prev = path[i - 1]
            p_curr = path[i]
            p_next = path[i + 1]
            
            # Tính góc quay
            angle = self.calculate_turn_angle(p_prev, p_curr, p_next)
            angle_deg = np.degrees(angle)
            
            # Nếu góc quay lớn, thêm điểm phụ
            if angle_deg > angle_threshold_deg:
                extra_points = self.add_control_point_by_angle(p_prev, p_curr, p_next)
                new_path.extend(extra_points)
            else:
                new_path.append(p_curr)
        
        new_path.append(path[-1])  # Giữ điểm cuối
        
        return np.array(new_path)
    
    def fix_curvature_violation(self, p_prev: np.ndarray,
                               p_violate: np.ndarray,
                               p_next: np.ndarray) -> List[np.ndarray]:
        """
        Sửa vi phạm ràng buộc độ cong bằng cách thêm điểm
        
        Phương pháp:
        - Tạo tam giác cân với khoảng cách = 1/2 grid size
        - Thay thế điểm vi phạm bằng 2 điểm mới
        - Chuyển góc tù thành 2 góc nhọn
        
        Args:
            p_prev: Điểm trước
            p_violate: Điểm vi phạm
            p_next: Điểm sau
            
        Returns:
            Danh sách 2 điểm mới thay thế điểm vi phạm
        """
        # Khoảng cách từ điểm uốn
        offset_distance = self.grid_size / 2.0
        
        # Vector từ prev đến violate
        v1 = p_violate - p_prev
        v1_normalized = v1 / (np.linalg.norm(v1) + 1e-10)
        
        # Vector từ violate đến next
        v2 = p_next - p_violate
        v2_normalized = v2 / (np.linalg.norm(v2) + 1e-10)
        
        # Tạo 2 điểm mới
        new_point_1 = p_violate - v1_normalized * offset_distance
        new_point_2 = p_violate + v2_normalized * offset_distance
        
        return [new_point_1, new_point_2]
    
    def optimize_control_points(self, path: np.ndarray,
                               ackermann: AckermannModel,
                               max_iterations: int = 10) -> Tuple[np.ndarray, bool]:
        """
        Tối ưu hóa điểm kiểm soát để thỏa mãn ràng buộc
        
        Args:
            path: Đường đi ban đầu
            ackermann: Mô hình Ackermann
            max_iterations: Số lần lặp tối đa
            
        Returns:
            Tuple (optimized_path, success):
            - optimized_path: Đường đi đã tối ưu
            - success: True nếu thỏa mãn ràng buộc
        """
        bspline = BSpline()
        current_path = path.copy()
        
        for iteration in range(max_iterations):
            # Kiểm tra ràng buộc độ cong
            is_valid, violated_segments = bspline.check_curvature_constraint(
                current_path, ackermann, n_samples=30
            )
            
            if is_valid:
                print(f"  ✓ Tối ưu thành công sau {iteration} lần lặp")
                return current_path, True
            
            # Thêm điểm để sửa vi phạm
            # Tính lại với chiến lược thêm điểm trung gian
            current_path = self.insert_intermediate_points(
                current_path, angle_threshold_deg=30.0
            )
        
        print(f"  ⚠ Chưa tối ưu hoàn toàn sau {max_iterations} lần lặp")
        return current_path, False


class BSplineSmoothing:
    """
    Lớp làm mượt đường đi bằng B-spline
    
    Kết hợp:
    - Tối ưu hóa điểm kiểm soát
    - Tạo đường cong B-spline
    - Kiểm tra ràng buộc Ackermann
    """
    
    def __init__(self, ackermann: AckermannModel, grid_size: float = 1.0):
        """
        Khởi tạo B-Spline Smoothing
        
        Args:
            ackermann: Mô hình Ackermann
            grid_size: Kích thước grid
        """
        self.ackermann = ackermann
        self.grid_size = grid_size
        self.bspline = BSpline()
        self.optimizer = ControlPointOptimizer(grid_size)
        
        print(f"✓ Khởi tạo B-Spline Smoothing")
        print(f"  - Ackermann L: {ackermann.L:.2f} m")
        print(f"  - φ_max: {np.degrees(ackermann.phi_max):.2f}°")
        print(f"  - ρ_max: {ackermann.rho_max:.6f}")
    
    def smooth_path(self, path_coords: np.ndarray,
                   n_samples: int = 100,
                   optimize: bool = True) -> dict:
        """
        Làm mượt đường đi
        
        Args:
            path_coords: Tọa độ đường đi ban đầu (Nx2)
            n_samples: Số điểm lấy mẫu trên mỗi segment
            optimize: Có tối ưu hóa điểm kiểm soát không
            
        Returns:
            Dictionary chứa kết quả:
            - smooth_path: Đường đi mượt
            - curvatures: Độ cong
            - control_points: Điểm kiểm soát
            - is_valid: Có thỏa mãn ràng buộc không
        """
        # Tối ưu hóa điểm kiểm soát nếu cần
        if optimize:
            print("Đang tối ưu hóa điểm kiểm soát...")
            control_points, is_optimized = self.optimizer.optimize_control_points(
                path_coords, self.ackermann, max_iterations=10
            )
        else:
            control_points = path_coords
            is_optimized = False
        
        # Thêm điểm trung gian
        print("Đang thêm điểm trung gian...")
        control_points = self.optimizer.insert_intermediate_points(
            control_points, angle_threshold_deg=45.0
        )
        
        print(f"Số điểm kiểm soát: {len(path_coords)} → {len(control_points)}")
        
        # Tạo đường cong B-spline
        if len(control_points) >= 4:
            print("Đang tạo đường cong B-spline...")
            smooth_path, curvatures = self.bspline.generate_curve(
                control_points, n_samples=n_samples
            )
            
            # Kiểm tra ràng buộc
            is_valid, violated = self.bspline.check_curvature_constraint(
                control_points, self.ackermann, n_samples=50
            )
            
            print(f"✓ Tạo đường mượt: {len(smooth_path)} điểm")
            print(f"  Độ cong max: {np.max(curvatures):.6f}")
            print(f"  Ràng buộc: {'Thỏa mãn ✓' if is_valid else 'Vi phạm ✗'}")
            
            return {
                'smooth_path': smooth_path,
                'curvatures': curvatures,
                'control_points': control_points,
                'is_valid': is_valid,
                'violated_segments': violated,
                'max_curvature': np.max(curvatures),
                'mean_curvature': np.mean(curvatures)
            }
        else:
            print("✗ Không đủ điểm kiểm soát (cần ≥ 4)")
            return {
                'smooth_path': path_coords,
                'curvatures': np.zeros(len(path_coords)),
                'control_points': control_points,
                'is_valid': False,
                'violated_segments': [],
                'max_curvature': 0.0,
                'mean_curvature': 0.0
            }


if __name__ == "__main__":
    # Test Control Point Optimizer
    print("Test Control Point Optimizer\n")
    
    # Tạo đường đi test
    path = np.array([
        [0, 0],
        [2, 0],
        [4, 2],
        [6, 4],
        [8, 4],
        [10, 2]
    ])
    
    # Test optimizer
    optimizer = ControlPointOptimizer(grid_size=1.0)
    
    print("\nTest thêm điểm trung gian:")
    new_path = optimizer.insert_intermediate_points(path, angle_threshold_deg=30.0)
    print(f"  Số điểm: {len(path)} → {len(new_path)}")
    
    # Test B-Spline Smoothing
    print("\n" + "="*50)
    print("Test B-Spline Smoothing")
    print("="*50)
    
    ackermann = AckermannModel(L=2.5, phi_max_deg=30.0)
    smoother = BSplineSmoothing(ackermann, grid_size=1.0)
    
    result = smoother.smooth_path(path, n_samples=50, optimize=True)
    
    print(f"\nKết quả:")
    print(f"  Số điểm mượt: {len(result['smooth_path'])}")
    print(f"  Độ cong max: {result['max_curvature']:.6f}")
    print(f"  Thỏa mãn ràng buộc: {result['is_valid']}")
