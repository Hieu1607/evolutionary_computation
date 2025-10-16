"""
Debug Test for IACO Algorithm
Kiểm tra chi tiết lỗi đi xuyên chướng ngại vật

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.environment.grid_map import GridMap
from src.aco.iaco import ImprovedACO
from src.create_paper_map import create_map_from_matrix
from src.visualization.plotter import PathPlotter


def check_path_collision(grid_map: GridMap, path_nodes: list) -> dict:
    """
    Kiểm tra xem đường đi có va chạm với chướng ngại vật không
    
    Returns:
        dict với collision_count, collision_nodes, và chi tiết
    """
    collisions = []
    
    for i, node in enumerate(path_nodes):
        # Chuyển node sang matrix position
        row, col = grid_map.grid_index_to_matrix_pos(node)
        
        # Kiểm tra xem có phải obstacle không
        if grid_map.grid[row, col] == 1:
            coord = grid_map.grid_index_to_coord(node)
            collisions.append({
                'step': i,
                'node': node,
                'position': (row, col),
                'coord': coord
            })
    
    return {
        'collision_count': len(collisions),
        'collisions': collisions,
        'is_valid': len(collisions) == 0
    }


def check_neighbors_validity(grid_map: GridMap, node: int) -> dict:
    """
    Kiểm tra xem các neighbor của node có đúng không
    """
    neighbors = grid_map.get_neighbors(node)
    valid_neighbors = []
    invalid_neighbors = []
    
    for neighbor in neighbors:
        row, col = grid_map.grid_index_to_matrix_pos(neighbor)
        if grid_map.grid[row, col] == 1:
            invalid_neighbors.append({
                'node': neighbor,
                'position': (row, col),
                'reason': 'Is obstacle'
            })
        else:
            valid_neighbors.append(neighbor)
    
    return {
        'total': len(neighbors),
        'valid': len(valid_neighbors),
        'invalid': len(invalid_neighbors),
        'valid_neighbors': valid_neighbors,
        'invalid_neighbors': invalid_neighbors
    }


def visualize_collision_details(grid_map: GridMap, path_nodes: list, 
                                 collision_info: dict, save_path: str = None):
    """
    Vẽ chi tiết các điểm va chạm
    """
    fig, ax = plt.subplots(figsize=(15, 15))
    
    # Vẽ grid
    ax.set_xlim(0, grid_map.x_max)
    ax.set_ylim(0, grid_map.y_max)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)', fontsize=12)
    ax.set_ylabel('y (m)', fontsize=12)
    ax.set_title('IACO Path Debug - Collision Detection', 
                fontsize=14, fontweight='bold')
    
    # Vẽ chướng ngại vật
    for row in range(grid_map.N_y):
        for col in range(grid_map.N_x):
            if grid_map.grid[row, col] == 1:
                x = col * grid_map.a
                y = (grid_map.N_y - row - 1) * grid_map.a
                rect = Rectangle((x, y), grid_map.a, grid_map.a,
                               facecolor='black', edgecolor='gray', alpha=0.8)
                ax.add_patch(rect)
    
    # Vẽ đường đi
    path_coords = [grid_map.grid_index_to_coord(node) for node in path_nodes]
    path_coords = np.array(path_coords)
    ax.plot(path_coords[:, 0], path_coords[:, 1], 
           'b-', linewidth=2, label='IACO Path', alpha=0.6)
    
    # Đánh dấu điểm va chạm bằng X đỏ lớn
    for collision in collision_info['collisions']:
        x, y = collision['coord']
        ax.plot(x, y, 'rX', markersize=20, markeredgewidth=4,
               label='Collision' if collision == collision_info['collisions'][0] else '')
        
        # Vẽ vòng tròn đỏ bao quanh
        circle = plt.Circle((x, y), 0.3, color='red', fill=False, 
                          linewidth=3, linestyle='--')
        ax.add_patch(circle)
        
        # Ghi số thứ tự
        ax.text(x + 0.4, y + 0.4, f"#{collision['step']}", 
               fontsize=10, color='red', fontweight='bold',
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Vẽ start và goal
    if len(path_nodes) > 0:
        start_coord = grid_map.grid_index_to_coord(path_nodes[0])
        goal_coord = grid_map.grid_index_to_coord(path_nodes[-1])
        
        ax.plot(start_coord[0], start_coord[1], 'go', markersize=15, 
               markeredgewidth=3, markeredgecolor='black', label='Start', zorder=10)
        ax.plot(goal_coord[0], goal_coord[1], 'r*', markersize=20, 
               markeredgewidth=2, markeredgecolor='black', label='Goal', zorder=10)
    
    ax.legend(loc='upper right', fontsize=12)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Đã lưu hình debug: {save_path}")
    
    plt.show()


def debug_ant_construction(grid_map: GridMap, start: int, goal: int, 
                          params: dict, max_steps: int = 5):
    """
    Debug chi tiết quá trình construct_solution của kiến
    """
    from src.aco.ant import Ant
    from src.aco.heuristic import HeuristicFunction
    from src.aco.pheromone import PheromoneMatrix
    
    print("\n" + "="*70)
    print("DEBUG: CHI TIẾT QUÁ TRÌNH KIẾN XÂY DỰNG ĐƯỜNG ĐI")
    print("="*70)
    
    # Khởi tạo
    heuristic = HeuristicFunction(
        lambda1=params['lambda1'],
        lambda2=params['lambda2'],
        P=params['P']
    )
    
    pheromone = PheromoneMatrix(
        n_nodes=grid_map.total_grids,
        rho=params['rho'],
        Q=params['Q']
    )
    
    ant = Ant(
        grid_map=grid_map,
        heuristic=heuristic,
        alpha=params['alpha'],
        beta=params['beta']
    )
    
    # Khởi tạo kiến
    ant.initialize(start, goal)
    current = start
    
    print(f"\nStart node: {start}")
    start_row, start_col = grid_map.grid_index_to_matrix_pos(start)
    print(f"  Position: row={start_row}, col={start_col}")
    print(f"  Coord: {grid_map.grid_index_to_coord(start)}")
    print(f"  Is obstacle: {grid_map.grid[start_row, start_col] == 1}")
    
    print(f"\nGoal node: {goal}")
    goal_row, goal_col = grid_map.grid_index_to_matrix_pos(goal)
    print(f"  Position: row={goal_row}, col={goal_col}")
    print(f"  Coord: {grid_map.grid_index_to_coord(goal)}")
    print(f"  Is obstacle: {grid_map.grid[goal_row, goal_col] == 1}")
    
    # Simulate từng bước
    for step in range(max_steps):
        print(f"\n{'='*70}")
        print(f"STEP {step + 1}: Current node = {current}")
        print(f"{'='*70}")
        
        # Lấy neighbors
        neighbors = grid_map.get_neighbors(current)
        print(f"Neighbors found: {len(neighbors)} nodes")
        
        # Kiểm tra từng neighbor
        for i, neighbor in enumerate(neighbors):
            row, col = grid_map.grid_index_to_matrix_pos(neighbor)
            coord = grid_map.grid_index_to_coord(neighbor)
            is_obstacle = grid_map.grid[row, col] == 1
            is_visited = neighbor in ant.visited
            
            print(f"  [{i+1}] Node {neighbor}:")
            print(f"      Position: ({row}, {col})")
            print(f"      Coord: {coord}")
            print(f"      Obstacle: {is_obstacle} {'❌' if is_obstacle else '✓'}")
            print(f"      Visited: {is_visited} {'❌' if is_visited else '✓'}")
        
        # Lọc allowed
        allowed = [n for n in neighbors if n not in ant.visited]
        print(f"\nAllowed neighbors (unvisited): {len(allowed)}")
        
        if len(allowed) == 0:
            print("⚠️ KHÔNG CÒN NODE NÀO ĐỂ DI CHUYỂN!")
            break
        
        # Chọn next node
        next_node = ant.select_next_node(current, pheromone.pheromone)
        
        if next_node is None:
            print("❌ KHÔNG THỂ CHỌN NODE TIẾP THEO!")
            break
        
        print(f"\n✓ Đã chọn: Node {next_node}")
        next_row, next_col = grid_map.grid_index_to_matrix_pos(next_node)
        print(f"  Position: ({next_row}, {next_col})")
        print(f"  Coord: {grid_map.grid_index_to_coord(next_node)}")
        
        # KIỂM TRA QUAN TRỌNG
        if grid_map.grid[next_row, next_col] == 1:
            print("🚨🚨🚨 LỖI NGHIÊM TRỌNG: KIẾN CHỌN VÀO CHƯỚNG NGẠI VẬT! 🚨🚨🚨")
            
            # Debug chi tiết
            print("\nDebug info:")
            print(f"  - Node {next_node} is in allowed list: {next_node in allowed}")
            print(f"  - grid_map.grid[{next_row}, {next_col}] = {grid_map.grid[next_row, next_col]}")
            
            # Kiểm tra get_neighbors
            print(f"\n  Checking get_neighbors logic...")
            neighbor_check = check_neighbors_validity(grid_map, current)
            print(f"    Total neighbors: {neighbor_check['total']}")
            print(f"    Valid: {neighbor_check['valid']}")
            print(f"    Invalid (obstacles): {neighbor_check['invalid']}")
            if neighbor_check['invalid'] > 0:
                print(f"    Invalid neighbors detail:")
                for inv in neighbor_check['invalid_neighbors']:
                    print(f"      - Node {inv['node']} at {inv['position']}: {inv['reason']}")
            
            break
        
        # Di chuyển
        ant.path.append(next_node)
        ant.visited.add(next_node)
        current = next_node
        
        if current == goal:
            print("\n✓✓✓ ĐÃ ĐẾN GOAL!")
            break


def main():
    """Main debug function"""
    print("="*70)
    print(" "*20 + "DEBUG IACO ALGORITHM")
    print("="*70)
    
    # Tạo bản đồ
    print("\n1. Tạo bản đồ...")
    grid_map, start_node, goal_node, start_coord, goal_coord = create_map_from_matrix(grid_size=1.0)
    
    # Kiểm tra start và goal
    print("\n2. Kiểm tra Start và Goal...")
    start_row, start_col = grid_map.grid_index_to_matrix_pos(start_node)
    goal_row, goal_col = grid_map.grid_index_to_matrix_pos(goal_node)
    
    print(f"Start: node {start_node}")
    print(f"  - Position: ({start_row}, {start_col})")
    print(f"  - Coord: {start_coord}")
    print(f"  - Is obstacle: {grid_map.grid[start_row, start_col] == 1}")
    
    print(f"\nGoal: node {goal_node}")
    print(f"  - Position: ({goal_row}, {goal_col})")
    print(f"  - Coord: {goal_coord}")
    print(f"  - Is obstacle: {grid_map.grid[goal_row, goal_col] == 1}")
    
    # Tham số
    params = {
        'n_iterations': 10,
        'n_ants': 5,
        'alpha': 1.0,
        'beta': 7.0,
        'rho': 0.3,
        'Q': 100.0,
        'lambda1': 0.6,
        'lambda2': 0.4,
        'P': 3.0,
        'k_L': 0.7,
        'k_E': 0.3
    }
    
    # Debug chi tiết quá trình construct
    print("\n3. Debug quá trình kiến xây dựng đường đi...")
    debug_ant_construction(grid_map, start_node, goal_node, params, max_steps=10)
    
    # Chạy IACO đầy đủ
    print("\n4. Chạy IACO đầy đủ...")
    iaco = ImprovedACO(
        grid_map=grid_map,
        start=start_node,
        goal=goal_node,
        **params
    )
    
    results = iaco.run(verbose=False)
    
    if results['success']:
        print(f"\n✓ IACO tìm thấy đường đi!")
        print(f"  - Số nodes: {results['n_nodes']}")
        print(f"  - Độ dài: {results['length']:.4f} m")
        
        # Kiểm tra va chạm
        print("\n5. Kiểm tra va chạm với chướng ngại vật...")
        collision_info = check_path_collision(grid_map, results['path'])
        
        if collision_info['is_valid']:
            print("✓✓✓ ĐƯỜNG ĐI HỢP LỆ - KHÔNG VA CHẠM!")
        else:
            print(f"❌❌❌ PHÁT HIỆN {collision_info['collision_count']} VA CHẠM!")
            print("\nChi tiết va chạm:")
            for col in collision_info['collisions']:
                print(f"  Step {col['step']}: Node {col['node']} tại {col['position']}")
                print(f"    Coord: {col['coord']}")
        
        # Vẽ visualization
        print("\n6. Tạo hình visualization...")
        visualize_collision_details(
            grid_map, 
            results['path'], 
            collision_info,
            save_path='debug_collision.png'
        )
        
    else:
        print("❌ IACO không tìm thấy đường đi!")
    
    print("\n" + "="*70)
    print("HOÀN THÀNH DEBUG")
    print("="*70)


if __name__ == "__main__":
    main()
