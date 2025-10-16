"""
Test Path Smoothing Module
Kiểm tra module làm mượt đường đi B-spline

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import sys
import os
import numpy as np

# Thêm đường dẫn
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ackermann.ackermann_model import AckermannModel
from src.path_smoothing.bspline import BSpline
from src.path_smoothing.control_point_optimizer import ControlPointOptimizer, BSplineSmoothing


def test_ackermann_model():
    """Test mô hình Ackermann"""
    print("\n" + "="*60)
    print("TEST MÔ HÌNH ACKERMANN")
    print("="*60)
    
    # Tạo mô hình
    model = AckermannModel(L=2.5, phi_max_deg=30.0)
    
    # Test validate góc lái (công thức 29)
    print("\nTest ràng buộc góc lái |φ| ≤ φ_max:")
    phi_valid = np.radians(25)
    phi_invalid = np.radians(35)
    
    assert model.validate_steering_angle(phi_valid), "Góc 25° phải hợp lệ"
    assert not model.validate_steering_angle(phi_invalid), "Góc 35° phải vi phạm"
    print(f"  ✓ φ = 25° → Hợp lệ")
    print(f"  ✓ φ = 35° → Vi phạm")
    
    # Test bán kính quay (công thức 30)
    print("\nTest bán kính quay R = L/tan(φ):")
    R = model.calculate_turning_radius(model.phi_max)
    R_expected = model.L / np.tan(model.phi_max)
    assert abs(R - R_expected) < 0.001, "Sai công thức bán kính"
    print(f"  ✓ R = {R:.4f} m (với φ_max = 30°)")
    
    # Test độ cong tối đa (công thức 31)
    print("\nTest độ cong tối đa ρ_max = tan(φ_max)/L:")
    rho_max = model.rho_max
    rho_expected = np.tan(model.phi_max) / model.L
    assert abs(rho_max - rho_expected) < 0.000001, "Sai công thức độ cong"
    print(f"  ✓ ρ_max = {rho_max:.6f} m⁻¹")
    
    # Test validate độ cong (công thức 35)
    print("\nTest ràng buộc độ cong ρ(u) ≤ ρ_max:")
    rho_valid = rho_max * 0.8
    rho_invalid = rho_max * 1.2
    
    assert model.validate_curvature(rho_valid), "Độ cong nhỏ hơn phải hợp lệ"
    assert not model.validate_curvature(rho_invalid), "Độ cong lớn hơn phải vi phạm"
    print(f"  ✓ ρ = {rho_valid:.6f} → Hợp lệ")
    print(f"  ✓ ρ = {rho_invalid:.6f} → Vi phạm")
    
    print("\n✓ Test Ackermann Model PASSED!")


def test_bspline_basis():
    """Test hàm cơ sở B-spline"""
    print("\n" + "="*60)
    print("TEST HÀM CƠ SỞ B-SPLINE")
    print("="*60)
    
    bspline = BSpline()
    
    # Test tính chất partition of unity (tổng = 1)
    print("\nTest tính chất: Σ B_i,3(u) = 1")
    test_points = [0.0, 0.25, 0.5, 0.75, 1.0]
    
    for u in test_points:
        B0 = bspline.basis_function_0(u)
        B1 = bspline.basis_function_1(u)
        B2 = bspline.basis_function_2(u)
        B3 = bspline.basis_function_3(u)
        total = B0 + B1 + B2 + B3
        
        assert abs(total - 1.0) < 1e-10, f"Tổng tại u={u} phải = 1"
        print(f"  u={u:.2f}: Σ B_i,3 = {total:.10f} ✓")
    
    print("\n✓ Test B-spline Basis PASSED!")


def test_bspline_curvature():
    """Test tính độ cong"""
    print("\n" + "="*60)
    print("TEST ĐỘ CONG B-SPLINE")
    print("="*60)
    
    bspline = BSpline()
    
    # Tạo đường thẳng (độ cong = 0)
    print("\nTest 1: Đường thẳng (ρ ≈ 0)")
    P0 = np.array([0, 0])
    P1 = np.array([1, 0])
    P2 = np.array([2, 0])
    P3 = np.array([3, 0])
    
    rho = bspline.calculate_curvature(P0, P1, P2, P3, u=0.5)
    assert rho < 0.01, "Đường thẳng phải có độ cong ≈ 0"
    print(f"  Độ cong: {rho:.6f} ✓")
    
    # Tạo đường cong (độ cong > 0)
    print("\nTest 2: Đường cong (ρ > 0)")
    P0 = np.array([0, 0])
    P1 = np.array([1, 0])
    P2 = np.array([2, 1])
    P3 = np.array([3, 2])
    
    rho = bspline.calculate_curvature(P0, P1, P2, P3, u=0.5)
    assert rho > 0, "Đường cong phải có độ cong > 0"
    print(f"  Độ cong: {rho:.6f} ✓")
    
    print("\n✓ Test B-spline Curvature PASSED!")


def test_control_point_optimizer():
    """Test tối ưu điểm kiểm soát"""
    print("\n" + "="*60)
    print("TEST CONTROL POINT OPTIMIZER")
    print("="*60)
    
    optimizer = ControlPointOptimizer(grid_size=1.0)
    
    # Test tính góc quay
    print("\nTest tính góc quay:")
    p_prev = np.array([0, 0])
    p_curr = np.array([1, 0])
    p_next = np.array([1, 1])
    
    angle = optimizer.calculate_turn_angle(p_prev, p_curr, p_next)
    angle_deg = np.degrees(angle)
    
    assert 85 < angle_deg < 95, "Góc phải ≈ 90°"
    print(f"  Góc quay: {angle_deg:.2f}° ✓")
    
    # Test thêm điểm kiểm soát
    print("\nTest thêm điểm kiểm soát:")
    new_points = optimizer.add_control_point_by_angle(p_prev, p_curr, p_next)
    
    assert len(new_points) == 3, "Phải trả về 3 điểm"
    print(f"  Số điểm trả về: {len(new_points)} ✓")
    
    # Test thêm điểm trung gian
    print("\nTest thêm điểm trung gian:")
    path = np.array([
        [0, 0],
        [2, 0],
        [2, 2],
        [4, 2]
    ])
    
    new_path = optimizer.insert_intermediate_points(path, angle_threshold_deg=45.0)
    assert len(new_path) >= len(path), "Số điểm phải tăng"
    print(f"  Số điểm: {len(path)} → {len(new_path)} ✓")
    
    print("\n✓ Test Control Point Optimizer PASSED!")


def test_bspline_smoothing():
    """Test làm mượt B-spline tích hợp"""
    print("\n" + "="*60)
    print("TEST B-SPLINE SMOOTHING")
    print("="*60)
    
    # Tạo mô hình Ackermann
    ackermann = AckermannModel(L=2.5, phi_max_deg=30.0)
    
    # Tạo smoother
    smoother = BSplineSmoothing(ackermann, grid_size=1.0)
    
    # Đường đi test
    path = np.array([
        [0, 0],
        [2, 0],
        [4, 1],
        [6, 3],
        [8, 5],
        [10, 6]
    ])
    
    print(f"\nĐường đi ban đầu: {len(path)} điểm")
    
    # Làm mượt
    result = smoother.smooth_path(path, n_samples=50, optimize=True)
    
    assert 'smooth_path' in result, "Phải có smooth_path"
    assert 'curvatures' in result, "Phải có curvatures"
    assert 'is_valid' in result, "Phải có is_valid"
    
    print(f"\nKết quả:")
    print(f"  Điểm kiểm soát: {len(result['control_points'])}")
    print(f"  Đường mượt: {len(result['smooth_path'])} điểm")
    print(f"  Độ cong max: {result['max_curvature']:.6f}")
    print(f"  Độ cong mean: {result['mean_curvature']:.6f}")
    print(f"  Thỏa mãn ràng buộc: {result['is_valid']}")
    
    print("\n✓ Test B-spline Smoothing PASSED!")


def main():
    """Chạy tất cả tests"""
    print("\n" + "="*60)
    print(" "*10 + "KIỂM TRA MODULE LÀM MƯỢT ĐƯỜNG ĐI")
    print("="*60)
    
    try:
        test_ackermann_model()
        test_bspline_basis()
        test_bspline_curvature()
        test_control_point_optimizer()
        test_bspline_smoothing()
        
        print("\n" + "="*60)
        print(" "*15 + "✓ TẤT CẢ TESTS ĐỀU PASS!")
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"\n✗ LỖI: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
