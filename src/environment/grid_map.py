"""
Grid Map Module
Mô đun xử lý lưới grid map cho môi trường robot

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
from typing import Tuple, List, Optional


class GridMap:
    """
    Lớp mô hình hóa môi trường dưới dạng lưới grid
    
    Attributes:
        a (float): Kích thước của mỗi grid (a × a)
        x_max (float): Chiều rộng tối đa của môi trường
        y_max (float): Chiều cao tối đa của môi trường
        N_x (int): Số grid theo hàng (trục x)
        N_y (int): Số grid theo cột (trục y)
        grid (np.ndarray): Ma trận grid (0: có thể đi, 1: chướng ngại vật)
    """
    
    def __init__(self, a: float, x_max: float, y_max: float):
        """
        Khởi tạo Grid Map
        
        Args:
            a: Kích thước của mỗi grid (a × a)
            x_max: Chiều rộng tối đa của môi trường
            y_max: Chiều cao tối đa của môi trường
        """
        self.a = a
        self.x_max = x_max
        self.y_max = y_max
        
        # Tính số grid theo công thức
        self.N_x = int(x_max / a)
        self.N_y = int(y_max / a)
        
        # Khởi tạo grid (tất cả đều có thể đi qua ban đầu)
        # 0: grid trắng (có thể đi qua)
        # 1: grid đen (chướng ngại vật)
        self.grid = np.zeros((self.N_y, self.N_x), dtype=int)
        
        # Tổng số grid
        self.total_grids = self.N_x * self.N_y
        
        print(f"✓ Khởi tạo Grid Map thành công!")
        print(f"  - Kích thước grid: {a} × {a}")
        print(f"  - Kích thước môi trường: {x_max} × {y_max}")
        print(f"  - Số grid: {self.N_x} × {self.N_y} = {self.total_grids}")
    
    def grid_index_to_coord(self, i: int) -> Tuple[float, float]:
        """
        Chuyển đổi từ số thứ tự grid sang tọa độ trung tâm
        
        Công thức (1):
        x_i = a × (mod(i, N_x) - 0.5)
        y_i = a × (N_y - ceil(i/N_x) + 0.5)
        
        Args:
            i: Số thứ tự của grid (bắt đầu từ 1)
            
        Returns:
            Tuple (x_i, y_i): Tọa độ trung tâm của grid thứ i
        """
        # Lưu ý: i bắt đầu từ 1 trong công thức
        # Fix: i % N_x = 0 khi i là bội của N_x, phải thành N_x
        col_index = i % self.N_x
        if col_index == 0:
            col_index = self.N_x
        
        x_i = self.a * (col_index - 0.5)
        y_i = self.a * (self.N_y - np.ceil(i / self.N_x) + 0.5)
        
        return (x_i, y_i)
    
    def coord_to_grid_index(self, x: float, y: float) -> int:
        """
        Chuyển đổi từ tọa độ sang số thứ tự grid
        
        Args:
            x: Tọa độ x
            y: Tọa độ y
            
        Returns:
            Số thứ tự grid (bắt đầu từ 1)
        """
        # Tính vị trí grid
        col = int(x / self.a)
        row = int((self.y_max - y) / self.a)
        
        # Kiểm tra biên
        col = max(0, min(col, self.N_x - 1))
        row = max(0, min(row, self.N_y - 1))
        
        # Chuyển về số thứ tự
        i = row * self.N_x + col + 1
        
        return i
    
    def grid_index_to_matrix_pos(self, i: int) -> Tuple[int, int]:
        """
        Chuyển số thứ tự grid sang vị trí trong ma trận
        
        Args:
            i: Số thứ tự grid (bắt đầu từ 1)
            
        Returns:
            Tuple (row, col): Vị trí trong ma trận grid
        """
        row = int((i - 1) / self.N_x)
        col = (i - 1) % self.N_x
        
        return (row, col)
    
    def matrix_pos_to_grid_index(self, row: int, col: int) -> int:
        """
        Chuyển vị trí ma trận sang số thứ tự grid
        
        Args:
            row: Hàng trong ma trận
            col: Cột trong ma trận
            
        Returns:
            Số thứ tự grid (bắt đầu từ 1)
        """
        return row * self.N_x + col + 1
    
    def add_obstacle(self, x: float, y: float, width: float = None, height: float = None):
        """
        Thêm chướng ngại vật vào grid map
        
        Args:
            x: Tọa độ x của góc trái dưới chướng ngại vật
            y: Tọa độ y của góc trái dưới chướng ngại vật
            width: Chiều rộng (mặc định = a)
            height: Chiều cao (mặc định = a)
        """
        if width is None:
            width = self.a
        if height is None:
            height = self.a
        
        # Tìm các grid bị ảnh hưởng
        col_start = int(x / self.a)
        col_end = int((x + width) / self.a)
        row_start = int((self.y_max - y - height) / self.a)
        row_end = int((self.y_max - y) / self.a)
        
        # Đảm bảo trong biên
        col_start = max(0, min(col_start, self.N_x - 1))
        col_end = max(0, min(col_end, self.N_x - 1))
        row_start = max(0, min(row_start, self.N_y - 1))
        row_end = max(0, min(row_end, self.N_y - 1))
        
        # Đánh dấu chướng ngại vật
        self.grid[row_start:row_end + 1, col_start:col_end + 1] = 1
    
    def add_obstacle_by_grid_index(self, i: int):
        """
        Thêm chướng ngại vật theo số thứ tự grid
        
        Args:
            i: Số thứ tự grid (bắt đầu từ 1)
        """
        row, col = self.grid_index_to_matrix_pos(i)
        if 0 <= row < self.N_y and 0 <= col < self.N_x:
            self.grid[row, col] = 1
    
    def add_obstacles_batch(self, grid_indices: List[int]):
        """
        Thêm nhiều chướng ngại vật cùng lúc
        
        Args:
            grid_indices: Danh sách các số thứ tự grid
        """
        for i in grid_indices:
            self.add_obstacle_by_grid_index(i)
        
        obstacle_count = np.sum(self.grid)
        print(f"✓ Đã thêm {len(grid_indices)} chướng ngại vật")
        print(f"  - Tổng số grid chướng ngại vật: {obstacle_count}/{self.total_grids}")
    
    def is_obstacle(self, i: int) -> bool:
        """
        Kiểm tra grid có phải chướng ngại vật không
        
        Args:
            i: Số thứ tự grid (bắt đầu từ 1)
            
        Returns:
            True nếu là chướng ngại vật, False nếu có thể đi qua
        """
        row, col = self.grid_index_to_matrix_pos(i)
        if 0 <= row < self.N_y and 0 <= col < self.N_x:
            return self.grid[row, col] == 1
        return True  # Ngoài biên coi như chướng ngại vật
    
    def is_coord_obstacle(self, x: float, y: float) -> bool:
        """
        Kiểm tra tọa độ có phải chướng ngại vật không
        
        Args:
            x: Tọa độ x
            y: Tọa độ y
            
        Returns:
            True nếu là chướng ngại vật, False nếu có thể đi qua
        """
        i = self.coord_to_grid_index(x, y)
        return self.is_obstacle(i)
    
    def get_neighbors(self, i: int, connectivity: int = 8) -> List[int]:
        """
        Lấy các grid láng giềng
        
        Args:
            i: Số thứ tự grid (bắt đầu từ 1)
            connectivity: 4 hoặc 8 (kết nối 4 hướng hoặc 8 hướng)
            
        Returns:
            Danh sách số thứ tự các grid láng giềng có thể đi qua
        """
        row, col = self.grid_index_to_matrix_pos(i)
        neighbors = []
        
        # 8 hướng: N, S, E, W, NE, NW, SE, SW
        if connectivity == 8:
            directions = [
                (-1, 0), (1, 0), (0, -1), (0, 1),  # N, S, W, E
                (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
            ]
        else:  # connectivity == 4
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # N, S, W, E
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            
            # Kiểm tra trong biên
            if 0 <= new_row < self.N_y and 0 <= new_col < self.N_x:
                # Kiểm tra không phải chướng ngại vật
                if self.grid[new_row, new_col] == 0:
                    neighbor_i = self.matrix_pos_to_grid_index(new_row, new_col)
                    neighbors.append(neighbor_i)
        
        return neighbors
    
    def clear_obstacles(self):
        """Xóa tất cả chướng ngại vật"""
        self.grid = np.zeros((self.N_y, self.N_x), dtype=int)
        print("✓ Đã xóa tất cả chướng ngại vật")
    
    def get_info(self) -> dict:
        """
        Lấy thông tin về Grid Map
        
        Returns:
            Dictionary chứa thông tin về grid map
        """
        obstacle_count = np.sum(self.grid)
        free_count = self.total_grids - obstacle_count
        
        return {
            'grid_size': self.a,
            'environment_size': (self.x_max, self.y_max),
            'grid_count': (self.N_x, self.N_y),
            'total_grids': self.total_grids,
            'obstacle_grids': int(obstacle_count),
            'free_grids': int(free_count),
            'obstacle_percentage': (obstacle_count / self.total_grids * 100)
        }
    
    def print_info(self):
        """In thông tin về Grid Map"""
        info = self.get_info()
        print("\n" + "="*50)
        print("THÔNG TIN GRID MAP")
        print("="*50)
        print(f"Kích thước grid:         {info['grid_size']} × {info['grid_size']}")
        print(f"Kích thước môi trường:   {info['environment_size'][0]} × {info['environment_size'][1]}")
        print(f"Số grid:                 {info['grid_count'][0]} × {info['grid_count'][1]} = {info['total_grids']}")
        print(f"Grid trống:              {info['free_grids']}")
        print(f"Grid chướng ngại vật:    {info['obstacle_grids']}")
        print(f"Tỷ lệ chướng ngại vật:   {info['obstacle_percentage']:.2f}%")
        print("="*50 + "\n")


if __name__ == "__main__":
    # Test GridMap
    print("Test GridMap Module\n")
    
    # Tạo grid map 100x100 với grid size 1
    grid_map = GridMap(a=1.0, x_max=100.0, y_max=100.0)
    
    # Test chuyển đổi grid index sang tọa độ
    print("\nTest chuyển đổi grid index sang tọa độ:")
    for i in [1, 10, 100, 1000]:
        x, y = grid_map.grid_index_to_coord(i)
        print(f"  Grid {i:4d} -> ({x:6.1f}, {y:6.1f})")
    
    # Test thêm chướng ngại vật
    print("\nTest thêm chướng ngại vật:")
    grid_map.add_obstacle(10, 10, 5, 5)
    grid_map.add_obstacles_batch([500, 501, 502, 600, 601, 602])
    
    # In thông tin
    grid_map.print_info()
