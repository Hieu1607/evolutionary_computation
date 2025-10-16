"""
B-Spline Module
Mô-đun đường cong B-spline bậc 3

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


class BSpline:
    """
    Lớp đường cong B-spline bậc 3
    
    Công thức (32-34):
    - P(u) = Σ P_i × B_{i,3}(u)
    - Hàm cơ sở B-spline bậc 3
    - Độ cong ρ(u) = |P''(u) × P'(u)| / |P'(u)|³
    """
    
    def __init__(self):
        """Khởi tạo B-Spline bậc 3"""
        self.degree = 3
    
    @staticmethod
    def basis_function_0(u: float) -> float:
        """
        Hàm cơ sở B-spline bậc 3 - B_{0,3}(u)
        
        Công thức (33):
        B_{0,3}(u) = (1/6) × (-u³ + 3u² - 3u + 1)
        
        Args:
            u: Tham số trong [0, 1]
            
        Returns:
            Giá trị hàm cơ sở
        """
        return (1.0 / 6.0) * (-u**3 + 3*u**2 - 3*u + 1)
    
    @staticmethod
    def basis_function_1(u: float) -> float:
        """
        Hàm cơ sở B-spline bậc 3 - B_{1,3}(u)
        
        Công thức (33):
        B_{1,3}(u) = (1/6) × (3u³ - 6u² + 4)
        
        Args:
            u: Tham số trong [0, 1]
            
        Returns:
            Giá trị hàm cơ sở
        """
        return (1.0 / 6.0) * (3*u**3 - 6*u**2 + 4)
    
    @staticmethod
    def basis_function_2(u: float) -> float:
        """
        Hàm cơ sở B-spline bậc 3 - B_{2,3}(u)
        
        Công thức (33):
        B_{2,3}(u) = (1/6) × (-3u³ + 3u² + 3u + 1)
        
        Args:
            u: Tham số trong [0, 1]
            
        Returns:
            Giá trị hàm cơ sở
        """
        return (1.0 / 6.0) * (-3*u**3 + 3*u**2 + 3*u + 1)
    
    @staticmethod
    def basis_function_3(u: float) -> float:
        """
        Hàm cơ sở B-spline bậc 3 - B_{3,3}(u)
        
        Công thức (33):
        B_{3,3}(u) = (1/6) × u³
        
        Args:
            u: Tham số trong [0, 1]
            
        Returns:
            Giá trị hàm cơ sở
        """
        return (1.0 / 6.0) * u**3
    
    def evaluate_segment(self, P0: np.ndarray, P1: np.ndarray,
                        P2: np.ndarray, P3: np.ndarray,
                        u: float) -> np.ndarray:
        """
        Tính điểm trên đường cong B-spline với 4 điểm kiểm soát
        
        Công thức (32):
        P(u) = Σ(i=0 đến 3) P_i × B_{i,3}(u)
        
        Args:
            P0, P1, P2, P3: 4 điểm kiểm soát liên tiếp
            u: Tham số trong [0, 1]
            
        Returns:
            Điểm trên đường cong
        """
        B0 = self.basis_function_0(u)
        B1 = self.basis_function_1(u)
        B2 = self.basis_function_2(u)
        B3 = self.basis_function_3(u)
        
        P = B0 * P0 + B1 * P1 + B2 * P2 + B3 * P3
        return P
    
    def evaluate_derivative_1(self, P0: np.ndarray, P1: np.ndarray,
                             P2: np.ndarray, P3: np.ndarray,
                             u: float) -> np.ndarray:
        """
        Tính đạo hàm bậc 1 P'(u)
        
        Args:
            P0, P1, P2, P3: 4 điểm kiểm soát
            u: Tham số trong [0, 1]
            
        Returns:
            Đạo hàm bậc 1
        """
        # Đạo hàm của hàm cơ sở
        B0_prime = (1.0 / 6.0) * (-3*u**2 + 6*u - 3)
        B1_prime = (1.0 / 6.0) * (9*u**2 - 12*u)
        B2_prime = (1.0 / 6.0) * (-9*u**2 + 6*u + 3)
        B3_prime = (1.0 / 6.0) * (3*u**2)
        
        P_prime = B0_prime * P0 + B1_prime * P1 + B2_prime * P2 + B3_prime * P3
        return P_prime
    
    def evaluate_derivative_2(self, P0: np.ndarray, P1: np.ndarray,
                             P2: np.ndarray, P3: np.ndarray,
                             u: float) -> np.ndarray:
        """
        Tính đạo hàm bậc 2 P''(u)
        
        Args:
            P0, P1, P2, P3: 4 điểm kiểm soát
            u: Tham số trong [0, 1]
            
        Returns:
            Đạo hàm bậc 2
        """
        # Đạo hàm bậc 2 của hàm cơ sở
        B0_double = (1.0 / 6.0) * (-6*u + 6)
        B1_double = (1.0 / 6.0) * (18*u - 12)
        B2_double = (1.0 / 6.0) * (-18*u + 6)
        B3_double = (1.0 / 6.0) * (6*u)
        
        P_double = B0_double * P0 + B1_double * P1 + B2_double * P2 + B3_double * P3
        return P_double
    
    def calculate_curvature(self, P0: np.ndarray, P1: np.ndarray,
                           P2: np.ndarray, P3: np.ndarray,
                           u: float) -> float:
        """
        Tính độ cong tại điểm u
        
        Công thức (34):
        ρ(u) = |P''(u) × P'(u)| / |P'(u)|³
        
        Args:
            P0, P1, P2, P3: 4 điểm kiểm soát
            u: Tham số trong [0, 1]
            
        Returns:
            Độ cong ρ(u)
        """
        # Tính đạo hàm bậc 1 và 2
        P_prime = self.evaluate_derivative_1(P0, P1, P2, P3, u)
        P_double = self.evaluate_derivative_2(P0, P1, P2, P3, u)
        
        # Tính tích có hướng (cross product) cho 2D
        # P' = [x', y'], P'' = [x'', y'']
        # P'' × P' = x'' * y' - y'' * x'
        cross = P_double[0] * P_prime[1] - P_double[1] * P_prime[0]
        
        # Tính độ dài P'
        P_prime_length = np.linalg.norm(P_prime)
        
        # Tránh chia cho 0
        if P_prime_length < 1e-10:
            return 0.0
        
        # Tính độ cong
        rho = abs(cross) / (P_prime_length ** 3)
        
        return rho
    
    def generate_curve(self, control_points: np.ndarray,
                      n_samples: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Tạo đường cong B-spline từ các điểm kiểm soát
        
        Args:
            control_points: Mảng Nx2 các điểm kiểm soát
            n_samples: Số điểm lấy mẫu trên mỗi segment
            
        Returns:
            Tuple (curve_points, curvatures):
            - curve_points: Các điểm trên đường cong
            - curvatures: Độ cong tại mỗi điểm
        """
        n_control = len(control_points)
        
        if n_control < 4:
            raise ValueError("Cần ít nhất 4 điểm kiểm soát")
        
        curve_points = []
        curvatures = []
        
        # Duyệt qua từng segment (4 điểm kiểm soát liên tiếp)
        for i in range(n_control - 3):
            P0 = control_points[i]
            P1 = control_points[i + 1]
            P2 = control_points[i + 2]
            P3 = control_points[i + 3]
            
            # Lấy mẫu trên segment này
            u_values = np.linspace(0, 1, n_samples)
            
            for u in u_values:
                # Tính điểm trên đường cong
                point = self.evaluate_segment(P0, P1, P2, P3, u)
                curve_points.append(point)
                
                # Tính độ cong
                rho = self.calculate_curvature(P0, P1, P2, P3, u)
                curvatures.append(rho)
        
        return np.array(curve_points), np.array(curvatures)
    
    def check_curvature_constraint(self, control_points: np.ndarray,
                                   ackermann: AckermannModel,
                                   n_samples: int = 50) -> Tuple[bool, List[int]]:
        """
        Kiểm tra ràng buộc độ cong
        
        Công thức (35):
        ρ(u) ≤ ρ_max
        
        Args:
            control_points: Các điểm kiểm soát
            ackermann: Mô hình Ackermann
            n_samples: Số điểm kiểm tra
            
        Returns:
            Tuple (is_valid, violated_segments):
            - is_valid: True nếu thỏa mãn ràng buộc
            - violated_segments: Danh sách các segment vi phạm
        """
        n_control = len(control_points)
        
        if n_control < 4:
            return True, []
        
        violated_segments = []
        
        # Kiểm tra từng segment
        for i in range(n_control - 3):
            P0 = control_points[i]
            P1 = control_points[i + 1]
            P2 = control_points[i + 2]
            P3 = control_points[i + 3]
            
            # Kiểm tra độ cong tại nhiều điểm
            u_values = np.linspace(0, 1, n_samples)
            
            for u in u_values:
                rho = self.calculate_curvature(P0, P1, P2, P3, u)
                
                # Kiểm tra vi phạm
                if not ackermann.validate_curvature(rho):
                    violated_segments.append(i)
                    break  # Đã tìm thấy vi phạm, chuyển sang segment tiếp theo
        
        is_valid = len(violated_segments) == 0
        return is_valid, violated_segments


if __name__ == "__main__":
    # Test B-Spline
    print("Test B-Spline Module\n")
    
    # Test hàm cơ sở
    print("Test hàm cơ sở B-spline tại u=0.5:")
    bspline = BSpline()
    u = 0.5
    
    B0 = bspline.basis_function_0(u)
    B1 = bspline.basis_function_1(u)
    B2 = bspline.basis_function_2(u)
    B3 = bspline.basis_function_3(u)
    
    print(f"  B_0,3({u}) = {B0:.6f}")
    print(f"  B_1,3({u}) = {B1:.6f}")
    print(f"  B_2,3({u}) = {B2:.6f}")
    print(f"  B_3,3({u}) = {B3:.6f}")
    print(f"  Tổng = {B0 + B1 + B2 + B3:.6f} (phải = 1.0)")
    
    # Test tạo đường cong
    print("\nTest tạo đường cong B-spline:")
    control_points = np.array([
        [0, 0],
        [1, 2],
        [3, 3],
        [5, 2],
        [6, 0]
    ])
    
    curve, curvatures = bspline.generate_curve(control_points, n_samples=20)
    print(f"  Số điểm kiểm soát: {len(control_points)}")
    print(f"  Số điểm trên đường cong: {len(curve)}")
    print(f"  Độ cong max: {np.max(curvatures):.6f}")
    print(f"  Độ cong mean: {np.mean(curvatures):.6f}")
    
    # Test kiểm tra ràng buộc
    print("\nTest kiểm tra ràng buộc độ cong:")
    ackermann = AckermannModel(L=2.5, phi_max_deg=30.0)
    
    is_valid, violated = bspline.check_curvature_constraint(
        control_points, ackermann, n_samples=30
    )
    
    print(f"  Ràng buộc: {'Thỏa mãn ✓' if is_valid else 'Vi phạm ✗'}")
    if not is_valid:
        print(f"  Các segment vi phạm: {violated}")
