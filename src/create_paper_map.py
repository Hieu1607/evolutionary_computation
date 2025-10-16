"""
Create Map from Paper
Tạo bản đồ giống như trong bài báo

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environment.grid_map import GridMap


def create_map_from_matrix(grid_size: float = 1.0) -> tuple:
    """
    Tạo bản đồ từ ma trận đã cho
    
    0: Ô trống
    1: Vật cản
    2: Điểm xuất phát
    3: Điểm kết thúc
    
    Args:
        grid_size: Kích thước mỗi grid
        
    Returns:
        Tuple (grid_map, start_node, goal_node, start_coord, goal_coord)
    """
    # Ma trận 20x20 (từ trên xuống dưới, trái sang phải)
    map_matrix = np.array([
        [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3]
    ])
    
    # Kích thước môi trường
    size = 20.0
    grid_map = GridMap(a=grid_size, x_max=size, y_max=size)
    
    # Tìm start và goal
    start_row, start_col = np.where(map_matrix == 2)
    goal_row, goal_col = np.where(map_matrix == 3)
    
    if len(start_row) == 0 or len(goal_row) == 0:
        raise ValueError("Không tìm thấy điểm start (2) hoặc goal (3) trong ma trận!")
    
    start_row, start_col = start_row[0], start_col[0]
    goal_row, goal_col = goal_row[0], goal_col[0]
    
    # Chuyển đổi sang node index (lưu ý: ma trận từ trên xuống)
    start_node = grid_map.matrix_pos_to_grid_index(start_row, start_col)
    goal_node = grid_map.matrix_pos_to_grid_index(goal_row, goal_col)
    
    # Lấy tọa độ
    start_coord = grid_map.grid_index_to_coord(start_node)
    goal_coord = grid_map.grid_index_to_coord(goal_node)
    
    # Thêm chướng ngại vật (1)
    obstacles = []
    obstacle_positions = np.where(map_matrix == 1)
    for row, col in zip(obstacle_positions[0], obstacle_positions[1]):
        node = grid_map.matrix_pos_to_grid_index(row, col)
        obstacles.append(node)
    
    # Thêm vào grid map
    if len(obstacles) > 0:
        grid_map.add_obstacles_batch(obstacles)
    
    print(f"✓ Đã tạo bản đồ từ ma trận 20×20")
    print(f"  - Số chướng ngại vật: {len(obstacles)}")
    print(f"  - Start: row={start_row}, col={start_col} → node {start_node} tại {start_coord}")
    print(f"  - Goal:  row={goal_row}, col={goal_col} → node {goal_node} tại {goal_coord}")
    
    return grid_map, start_node, goal_node, start_coord, goal_coord


def create_paper_map(size: float = 20.0, grid_size: float = 1.0) -> GridMap:
    """
    Tạo bản đồ giống như trong bài báo
    
    Dựa trên hình ảnh: Grid 20x20 với các chướng ngại vật phức tạp
    
    Args:
        size: Kích thước môi trường (20x20)
        grid_size: Kích thước mỗi grid (1.0)
        
    Returns:
        GridMap object
    """
    grid_map = GridMap(a=grid_size, x_max=size, y_max=size)
    
    # Phân tích từ hình ảnh bài báo
    # Các chướng ngại vật được mô tả theo vị trí (row, col, width, height)
    
    obstacles = []
    
    # Chướng ngại vật ở góc trên trái
    for i in range(18, 20):  # row 18-19 (gần trên)
        for j in range(0, 3):  # col 0-2
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật hàng trên
    for i in range(17, 19):
        for j in range(5, 8):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật ở phải trên
    for i in range(18, 20):
        for j in range(15, 17):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật góc phải trên
    for i in range(17, 19):
        for j in range(18, 20):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật giữa trái
    for i in range(14, 17):
        for j in range(0, 2):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật lớn ở giữa (hình chữ U)
    # Phần trái chữ U
    for i in range(11, 15):
        for j in range(5, 7):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Phần đáy chữ U
    for i in range(11, 13):
        for j in range(7, 10):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Phần phải chữ U
    for i in range(11, 15):
        for j in range(9, 11):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật ở giữa phải
    for i in range(13, 16):
        for j in range(14, 17):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật dưới trái
    for i in range(5, 8):
        for j in range(0, 3):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật hàng dưới giữa
    for i in range(6, 9):
        for j in range(8, 11):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật hình L dưới
    for i in range(4, 7):
        for j in range(13, 15):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    for i in range(6, 8):
        for j in range(15, 17):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật dưới phải
    for i in range(3, 6):
        for j in range(17, 20):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Chướng ngại vật dưới cùng
    for i in range(0, 3):
        for j in range(6, 9):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    for i in range(1, 3):
        for j in range(11, 14):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Góc dưới phải
    for i in range(0, 2):
        for j in range(17, 19):
            obstacles.append(grid_map.matrix_pos_to_grid_index(i, j))
    
    # Loại bỏ trùng lặp
    obstacles = list(set(obstacles))
    
    # Thêm vào grid map
    grid_map.add_obstacles_batch(obstacles)
    
    return grid_map


def get_paper_start_goal(grid_map: GridMap) -> tuple:
    """
    Lấy vị trí start và goal từ bài báo
    
    Args:
        grid_map: GridMap object
        
    Returns:
        Tuple (start_node, goal_node, start_coord, goal_coord)
    """
    # Từ hình ảnh: Start ở góc trên trái, Goal ở góc dưới phải
    start_coord = (1.0, 19.0)  # Gần góc trên trái
    goal_coord = (19.0, 1.0)    # Gần góc dưới phải
    
    start_node = grid_map.coord_to_grid_index(start_coord[0], start_coord[1])
    goal_node = grid_map.coord_to_grid_index(goal_coord[0], goal_coord[1])
    
    return start_node, goal_node, start_coord, goal_coord


if __name__ == "__main__":
    print("Tạo bản đồ từ bài báo...\n")
    
    # Tạo map
    grid_map = create_paper_map(size=20.0, grid_size=1.0)
    
    # Lấy start và goal
    start_node, goal_node, start_coord, goal_coord = get_paper_start_goal(grid_map)
    
    print(f"Start: node {start_node} tại {start_coord}")
    print(f"Goal: node {goal_node} tại {goal_coord}")
    
    # In thông tin
    grid_map.print_info()
    
    # Test vẽ
    try:
        import matplotlib.pyplot as plt
        from src.visualization.plotter import PathPlotter
        
        plotter = PathPlotter(grid_map, figsize=(10, 10))
        fig, ax = plotter.create_figure(title="Map from Paper")
        plotter.plot_start_goal(ax, start_coord, goal_coord)
        
        plt.savefig('paper_map.png', dpi=300, bbox_inches='tight')
        print("\n✓ Đã lưu bản đồ: paper_map.png")
        
        plt.show()
        
    except ImportError:
        print("\nChưa cài matplotlib, bỏ qua visualization")
