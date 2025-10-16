"""
Ackermann Model Module
Mô hình động học robot Ackermann

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import Tuple


class AckermannModel:
    """
    Mô hình động học robot Ackermann
    
    Attributes:
        L: Chiều dài trục bánh xe (wheelbase)
        phi_max: Góc lái tối đa (radian)
        R_min: Bán kính quay tối thiểu
        rho_max: Độ cong tối đa
    """
    
    def __init__(self, L: float = 2.5, phi_max_deg: float = 30.0):
        """
        Khởi tạo Ackermann Model
        
        Args:
            L: Chiều dài trục bánh xe (wheelbase) - mặc định 2.5m
            phi_max_deg: Góc lái tối đa theo độ - mặc định 30°
        """
        self.L = L
        self.phi_max = np.radians(phi_max_deg)  # Chuyển sang radian
        
        # Tính bán kính quay tối thiểu (công thức 30)
        self.R_min = self.calculate_turning_radius(self.phi_max)
        
        # Tính độ cong tối đa (công thức 31)
        self.rho_max = self.calculate_max_curvature()
        
        print(f"✓ Khởi tạo Ackermann Model")
        print(f"  - Wheelbase L: {L:.2f} m")
        print(f"  - Góc lái max φ_max: {phi_max_deg:.2f}° ({self.phi_max:.4f} rad)")
        print(f"  - Bán kính quay min R_min: {self.R_min:.4f} m")
        print(f"  - Độ cong max ρ_max: {self.rho_max:.6f} m⁻¹")
    
    def validate_steering_angle(self, phi: float) -> bool:
        """
        Kiểm tra góc lái có hợp lệ không
        
        Công thức (29):
        |φ| ≤ φ_max
        
        Args:
            phi: Góc lái (radian)
            
        Returns:
            True nếu hợp lệ, False nếu vi phạm
        """
        return abs(phi) <= self.phi_max
    
    def calculate_turning_radius(self, phi: float) -> float:
        """
        Tính bán kính quay
        
        Công thức (30):
        R = L / tan(φ)
        
        Args:
            phi: Góc lái (radian)
            
        Returns:
            Bán kính quay R
        """
        if abs(phi) < 1e-10:
            return float('inf')  # Đi thẳng
        
        R = self.L / np.tan(abs(phi))
        return R
    
    def calculate_max_curvature(self) -> float:
        """
        Tính độ cong tối đa
        
        Công thức (31):
        ρ_max = 1/R_min = tan(φ_max)/L
        
        Returns:
            Độ cong tối đa ρ_max
        """
        rho_max = np.tan(self.phi_max) / self.L
        return rho_max
    
    def validate_curvature(self, rho: float) -> bool:
        """
        Kiểm tra độ cong có hợp lệ không
        
        Công thức (35):
        ρ(u) ≤ ρ_max
        
        Args:
            rho: Độ cong cần kiểm tra
            
        Returns:
            True nếu hợp lệ, False nếu vi phạm
        """
        return abs(rho) <= self.rho_max
    
    def calculate_steering_angle_from_radius(self, R: float) -> float:
        """
        Tính góc lái từ bán kính quay
        
        Args:
            R: Bán kính quay
            
        Returns:
            Góc lái phi (radian)
        """
        if R == float('inf') or R > 1e10:
            return 0.0
        
        phi = np.arctan(self.L / R)
        return phi
    
    def calculate_steering_angle_from_curvature(self, rho: float) -> float:
        """
        Tính góc lái từ độ cong
        
        Args:
            rho: Độ cong
            
        Returns:
            Góc lái phi (radian)
        """
        if abs(rho) < 1e-10:
            return 0.0
        
        phi = np.arctan(rho * self.L)
        return phi
    
    def get_model_info(self) -> dict:
        """
        Lấy thông tin mô hình
        
        Returns:
            Dictionary chứa thông tin mô hình
        """
        return {
            'wheelbase_L': self.L,
            'phi_max_rad': self.phi_max,
            'phi_max_deg': np.degrees(self.phi_max),
            'R_min': self.R_min,
            'rho_max': self.rho_max
        }
    
    def print_info(self):
        """In thông tin mô hình"""
        info = self.get_model_info()
        
        print("\n" + "="*50)
        print("THÔNG TIN MÔ HÌNH ACKERMANN")
        print("="*50)
        print(f"Chiều dài trục bánh xe L:    {info['wheelbase_L']:.2f} m")
        print(f"Góc lái tối đa φ_max:        {info['phi_max_deg']:.2f}° ({info['phi_max_rad']:.4f} rad)")
        print(f"Bán kính quay tối thiểu:     {info['R_min']:.4f} m")
        print(f"Độ cong tối đa ρ_max:        {info['rho_max']:.6f} m⁻¹")
        print("="*50 + "\n")


if __name__ == "__main__":
    # Test Ackermann Model
    print("Test Ackermann Model\n")
    
    # Tạo mô hình
    model = AckermannModel(L=2.5, phi_max_deg=30.0)
    
    # Test validate góc lái
    print("\nTest validate góc lái:")
    phi_test = np.radians(25)
    valid = model.validate_steering_angle(phi_test)
    print(f"  φ = 25° → {'Hợp lệ' if valid else 'Vi phạm'}")
    
    phi_test = np.radians(35)
    valid = model.validate_steering_angle(phi_test)
    print(f"  φ = 35° → {'Hợp lệ' if valid else 'Vi phạm'}")
    
    # Test tính bán kính quay
    print("\nTest bán kính quay:")
    R = model.calculate_turning_radius(np.radians(30))
    print(f"  φ = 30° → R = {R:.4f} m")
    
    # Test validate độ cong
    print("\nTest validate độ cong:")
    rho_test = 0.2
    valid = model.validate_curvature(rho_test)
    print(f"  ρ = {rho_test:.4f} → {'Hợp lệ' if valid else 'Vi phạm'}")
    
    # In thông tin
    model.print_info()
