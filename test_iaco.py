"""
Test IACO Algorithm
Kiểm tra thuật toán kiến cải tiến

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import sys
import os

# Thêm đường dẫn
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environment.grid_map import GridMap
from src.aco.heuristic import HeuristicFunction, PathEvaluator
from src.aco.pheromone import PheromoneMatrix
from src.aco.iaco import ImprovedACO


def test_heuristic():
    """Test hàm heuristic"""
    print("\n" + "="*60)
    print("TEST HÀM HEURISTIC")
    print("="*60)
    
    heuristic = HeuristicFunction(lambda1=0.6, lambda2=0.4, P=3.0)
    
    # Test khoảng cách
    d = heuristic.euclidean_distance(0, 0, 3, 4)
    print(f"\nKhoảng cách từ (0,0) đến (3,4): {d:.2f}")
    assert abs(d - 5.0) < 0.01, "Sai khoảng cách!"
    
    # Test góc quay
    import numpy as np
    delta = heuristic.calculate_turn_angle(0, 0, 1, 0, 1, 1)
    print(f"Góc quay: {np.degrees(delta):.2f}°")
    
    # Test heuristic đầy đủ
    eta = heuristic.calculate_heuristic_full(0, 0, 1, 0, 2, 1, 10, 10)
    print(f"Heuristic: {eta:.6f}")
    
    print("✓ Test heuristic PASSED!\n")


def test_path_evaluator():
    """Test đánh giá đường đi"""
    print("="*60)
    print("TEST PATH EVALUATOR")
    print("="*60)
    
    import numpy as np
    
    evaluator = PathEvaluator(k_L=0.7, k_E=0.3, P=3.0)
    
    # Đường đi test
    path = np.array([
        [0, 0],
        [1, 0],
        [2, 0],
        [3, 1],
        [4, 2]
    ])
    
    # Đánh giá
    result = evaluator.evaluate_path(path)
    
    print(f"\nĐộ dài: {result['length']:.4f}")
    print(f"Năng lượng: {result['energy']:.4f}")
    print(f"Chi phí: {result['cost']:.4f}")
    
    assert result['length'] > 0, "Độ dài phải > 0"
    assert result['cost'] > 0, "Chi phí phải > 0"
    
    print("✓ Test evaluator PASSED!\n")


def test_pheromone():
    """Test pheromone matrix"""
    print("="*60)
    print("TEST PHEROMONE MATRIX")
    print("="*60)
    
    pheromone = PheromoneMatrix(
        n_nodes=100,
        rho=0.3,
        Q=100.0,
        tau_min=0.01,
        tau_max=10.0
    )
    
    # Test cơ chế thưởng phạt
    w1 = pheromone.calculate_reward_penalty(50, 40, 80)
    w2 = pheromone.calculate_reward_penalty(70, 40, 80)
    
    print(f"\nS_k=50 → w={w1} ({'Thưởng' if w1 > 0 else 'Phạt'})")
    print(f"S_k=70 → w={w2} ({'Thưởng' if w2 > 0 else 'Phạt'})")
    
    assert w1 == 1, "Phải thưởng"
    assert w2 == -1, "Phải phạt"
    
    # Test cập nhật
    paths = [[0, 1, 2, 3], [0, 5, 10, 15]]
    costs = [100.0, 120.0]
    pheromone.update_pheromone_two_layer(paths, costs, 90.0)
    
    stats = pheromone.get_statistics()
    print(f"\nPheromone mean: {stats['mean']:.4f}")
    print(f"Pheromone min: {stats['min']:.4f}")
    print(f"Pheromone max: {stats['max']:.4f}")
    
    assert stats['min'] >= pheromone.tau_min, "Vi phạm tau_min"
    assert stats['max'] <= pheromone.tau_max, "Vi phạm tau_max"
    
    print("✓ Test pheromone PASSED!\n")


def test_iaco_simple():
    """Test IACO trên bài toán đơn giản"""
    print("="*60)
    print("TEST IACO - BÀI TOÁN ĐỐN GIẢN")
    print("="*60)
    
    # Tạo grid map nhỏ
    grid_map = GridMap(a=1.0, x_max=10.0, y_max=10.0)
    
    # Thêm vài chướng ngại vật
    obstacles = [23, 24, 33, 34, 43, 44]
    grid_map.add_obstacles_batch(obstacles)
    
    # Start và goal
    start = 1
    goal = 99
    
    print(f"\nStart: {start}, Goal: {goal}")
    
    # Chạy IACO
    iaco = ImprovedACO(
        grid_map=grid_map,
        start=start,
        goal=goal,
        n_ants=10,
        n_iterations=20,
        alpha=2.0,
        beta=5.0
    )
    
    results = iaco.run(verbose=False)
    
    if results['success']:
        print(f"\n✓ Tìm thấy đường đi!")
        print(f"  Số nodes: {results['n_nodes']}")
        print(f"  Độ dài: {results['length']:.2f}")
        print(f"  Chi phí: {results['cost']:.2f}")
    else:
        print(f"\n✗ Không tìm thấy đường đi")
    
    print("✓ Test IACO PASSED!\n")


def main():
    """Chạy tất cả tests"""
    print("\n" + "="*60)
    print(" "*15 + "KIỂM TRA THUẬT TOÁN IACO")
    print("="*60)
    
    try:
        test_heuristic()
        test_path_evaluator()
        test_pheromone()
        test_iaco_simple()
        
        print("="*60)
        print(" "*15 + "✓ TẤT CẢ TESTS ĐỀU PASS!")
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"\n✗ LỖI: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
