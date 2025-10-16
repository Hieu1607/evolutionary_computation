"""
Path Plotter Module
Vẽ đồ thị đường đi, chướng ngại vật, B-spline

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
import matplotlib.patches as mpatches
from typing import Optional, Tuple, List
import sys
import os

# Thêm đường dẫn
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from environment.grid_map import GridMap


class PathPlotter:
    """
    Lớp vẽ đồ thị cho quy hoạch đường đi
    
    Tính năng:
    - Vẽ grid map và chướng ngại vật
    - Vẽ đường đi IACO (đỏ)
    - Vẽ đường đi B-spline mượt (xanh lá/xanh dương)
    - Vẽ điểm start/goal
    - Tô màu độ cong
    """
    
    def __init__(self, grid_map: GridMap, figsize: Tuple[int, int] = (10, 10)):
        """
        Khởi tạo Path Plotter
        
        Args:
            grid_map: Bản đồ lưới
            figsize: Kích thước figure
        """
        self.grid_map = grid_map
        self.figsize = figsize
        
        # Màu sắc theo bài báo
        self.color_start = '#00FF00'      # Xanh lá (start)
        self.color_goal = '#FF0000'       # Đỏ (goal)
        self.color_obstacle = '#000000'   # Đen (chướng ngại vật)
        self.color_grid = '#CCCCCC'       # Xám nhạt (lưới)
        self.color_iaco_path = '#FF0000'  # Đỏ (đường IACO)
        self.color_smooth_path = '#00FF00' # Xanh lá (đường B-spline)
        self.color_smooth_path_alt = '#0000FF' # Xanh dương (đường B-spline)
        
    def plot_grid(self, ax: plt.Axes, show_grid: bool = True):
        """
        Vẽ lưới grid
        
        Args:
            ax: Matplotlib axes
            show_grid: Hiển thị lưới hay không
        """
        if show_grid:
            # Vẽ lưới theo grid
            for i in range(self.grid_map.N_x + 1):
                x = i * self.grid_map.a
                ax.plot([x, x], [0, self.grid_map.y_max], 
                       color=self.color_grid, linewidth=0.5, alpha=0.3)
            
            for j in range(self.grid_map.N_y + 1):
                y = j * self.grid_map.a
                ax.plot([0, self.grid_map.x_max], [y, y], 
                       color=self.color_grid, linewidth=0.5, alpha=0.3)
    
    def plot_obstacles(self, ax: plt.Axes):
        """
        Vẽ chướng ngại vật (ô đen)
        
        Args:
            ax: Matplotlib axes
        """
        for row in range(self.grid_map.N_y):
            for col in range(self.grid_map.N_x):
                if self.grid_map.grid[row, col] == 1:
                    # Tính tọa độ góc dưới bên trái
                    x = col * self.grid_map.a
                    y = (self.grid_map.N_y - row - 1) * self.grid_map.a
                    
                    # Vẽ hình chữ nhật đen
                    rect = Rectangle((x, y), self.grid_map.a, self.grid_map.a,
                                   facecolor=self.color_obstacle,
                                   edgecolor='none')
                    ax.add_patch(rect)
    
    def plot_start_goal(self, ax: plt.Axes, start_coord: Tuple[float, float],
                       goal_coord: Tuple[float, float], size: float = 0.5):
        """
        Vẽ điểm start (xanh lá) và goal (đỏ)
        
        Args:
            ax: Matplotlib axes
            start_coord: Tọa độ điểm start (x, y)
            goal_coord: Tọa độ điểm goal (x, y)
            size: Kích thước marker
        """
        # Start - ô vuông xanh lá
        ax.add_patch(Rectangle(
            (start_coord[0] - size/2, start_coord[1] - size/2),
            size, size,
            facecolor=self.color_start,
            edgecolor='black',
            linewidth=2,
            zorder=10
        ))
        
        # Goal - ô vuông đỏ
        ax.add_patch(Rectangle(
            (goal_coord[0] - size/2, goal_coord[1] - size/2),
            size, size,
            facecolor=self.color_goal,
            edgecolor='black',
            linewidth=2,
            zorder=10
        ))
    
    def plot_path(self, ax: plt.Axes, path_coords: np.ndarray,
                 color: str, linewidth: float = 2.0, 
                 label: str = '', style: str = '-', alpha: float = 1.0):
        """
        Vẽ đường đi
        
        Args:
            ax: Matplotlib axes
            path_coords: Tọa độ đường đi (Nx2)
            color: Màu đường
            linewidth: Độ dày đường
            label: Nhãn cho legend
            style: Kiểu đường ('-', '--', ':')
            alpha: Độ trong suốt
        """
        if len(path_coords) < 2:
            return
        
        ax.plot(path_coords[:, 0], path_coords[:, 1],
               color=color, linewidth=linewidth, 
               linestyle=style, label=label, alpha=alpha,
               zorder=5)
    
    def plot_path_with_curvature(self, ax: plt.Axes, path_coords: np.ndarray,
                                curvatures: np.ndarray, linewidth: float = 3.0,
                                label: str = '', cmap: str = 'RdYlGn_r'):
        """
        Vẽ đường đi với màu theo độ cong
        
        Args:
            ax: Matplotlib axes
            path_coords: Tọa độ đường đi (Nx2)
            curvatures: Độ cong tại mỗi điểm
            linewidth: Độ dày đường
            label: Nhãn
            cmap: Colormap
        """
        if len(path_coords) < 2:
            return
        
        # Tạo segments
        points = path_coords.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Tạo LineCollection với màu theo độ cong
        lc = LineCollection(segments, cmap=cmap, linewidth=linewidth, zorder=5)
        lc.set_array(curvatures[:-1])
        lc.set_label(label)
        
        line = ax.add_collection(lc)
        
        return line
    
    def create_figure(self, title: str = "Path Planning",
                     show_grid: bool = True) -> Tuple[plt.Figure, plt.Axes]:
        """
        Tạo figure mới
        
        Args:
            title: Tiêu đề
            show_grid: Hiển thị lưới
            
        Returns:
            Tuple (fig, ax)
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Vẽ lưới
        if show_grid:
            self.plot_grid(ax, show_grid=True)
        
        # Vẽ chướng ngại vật
        self.plot_obstacles(ax)
        
        # Thiết lập axes
        ax.set_xlim(0, self.grid_map.x_max)
        ax.set_ylim(0, self.grid_map.y_max)
        ax.set_aspect('equal')
        ax.set_xlabel('x(m)', fontsize=12)
        ax.set_ylabel('y(m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        
        return fig, ax
    
    def plot_iaco_result(self, path_coords: np.ndarray,
                        start_coord: Tuple[float, float],
                        goal_coord: Tuple[float, float],
                        title: str = "IACO Path Planning",
                        save_path: Optional[str] = None):
        """
        Vẽ kết quả IACO (đường đỏ gấp khúc)
        
        Args:
            path_coords: Tọa độ đường đi IACO
            start_coord: Tọa độ start
            goal_coord: Tọa độ goal
            title: Tiêu đề
            save_path: Đường dẫn lưu file (nếu muốn)
        """
        fig, ax = self.create_figure(title=title, show_grid=True)
        
        # Vẽ đường đi IACO (đỏ)
        self.plot_path(ax, path_coords, 
                      color=self.color_iaco_path,
                      linewidth=2.0,
                      label='IACO Path',
                      style='-')
        
        # Vẽ start và goal
        self.plot_start_goal(ax, start_coord, goal_coord, size=0.6)
        
        # Legend
        ax.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✓ Đã lưu: {save_path}")
        
        return fig, ax
    
    def plot_smooth_result(self, smooth_coords: np.ndarray,
                          start_coord: Tuple[float, float],
                          goal_coord: Tuple[float, float],
                          curvatures: Optional[np.ndarray] = None,
                          title: str = "B-spline Smoothed Path",
                          save_path: Optional[str] = None,
                          show_curvature: bool = False):
        """
        Vẽ kết quả B-spline (đường xanh mượt)
        
        Args:
            smooth_coords: Tọa độ đường đi mượt
            start_coord: Tọa độ start
            goal_coord: Tọa độ goal
            curvatures: Độ cong (nếu muốn vẽ màu theo độ cong)
            title: Tiêu đề
            save_path: Đường dẫn lưu file
            show_curvature: Hiển thị màu theo độ cong
        """
        fig, ax = self.create_figure(title=title, show_grid=True)
        
        # Vẽ đường đi mượt
        if show_curvature and curvatures is not None:
            line = self.plot_path_with_curvature(
                ax, smooth_coords, curvatures,
                linewidth=3.0, label='B-spline Path',
                cmap='RdYlGn_r'
            )
            # Thêm colorbar
            cbar = plt.colorbar(line, ax=ax)
            cbar.set_label('Curvature ρ(u)', fontsize=10)
        else:
            self.plot_path(ax, smooth_coords,
                          color=self.color_smooth_path,
                          linewidth=2.5,
                          label='B-spline Path',
                          style='-')
        
        # Vẽ start và goal
        self.plot_start_goal(ax, start_coord, goal_coord, size=0.6)
        
        # Legend
        ax.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✓ Đã lưu: {save_path}")
        
        return fig, ax
    
    def plot_comparison(self, iaco_coords: np.ndarray,
                       smooth_coords: np.ndarray,
                       start_coord: Tuple[float, float],
                       goal_coord: Tuple[float, float],
                       title: str = "Path Comparison: IACO vs B-spline",
                       save_path: Optional[str] = None):
        """
        Vẽ so sánh IACO và B-spline
        
        Args:
            iaco_coords: Tọa độ đường IACO
            smooth_coords: Tọa độ đường B-spline
            start_coord: Tọa độ start
            goal_coord: Tọa độ goal
            title: Tiêu đề
            save_path: Đường dẫn lưu file
        """
        fig, ax = self.create_figure(title=title, show_grid=True)
        
        # Vẽ đường IACO (đỏ, nét đứt)
        self.plot_path(ax, iaco_coords,
                      color=self.color_iaco_path,
                      linewidth=2.0,
                      label='IACO Path',
                      style='--',
                      alpha=0.7)
        
        # Vẽ đường B-spline (xanh lá, nét liền)
        self.plot_path(ax, smooth_coords,
                      color=self.color_smooth_path,
                      linewidth=2.5,
                      label='B-spline Smoothed',
                      style='-')
        
        # Vẽ start và goal
        self.plot_start_goal(ax, start_coord, goal_coord, size=0.6)
        
        # Legend
        ax.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✓ Đã lưu: {save_path}")
        
        return fig, ax
    
    def plot_multi_comparison(self, results_list: List[dict],
                             titles: List[str],
                             suptitle: str = "Path Planning Results",
                             save_path: Optional[str] = None):
        """
        Vẽ nhiều kết quả cạnh nhau (như trong bài báo)
        
        Args:
            results_list: Danh sách kết quả
            titles: Danh sách tiêu đề
            suptitle: Tiêu đề chung
            save_path: Đường dẫn lưu file
        """
        n_plots = len(results_list)
        fig, axes = plt.subplots(1, n_plots, figsize=(8*n_plots, 8))
        
        if n_plots == 1:
            axes = [axes]
        
        for idx, (result, title) in enumerate(zip(results_list, titles)):
            ax = axes[idx]
            
            # Vẽ lưới
            self.plot_grid(ax, show_grid=True)
            
            # Vẽ chướng ngại vật
            self.plot_obstacles(ax)
            
            # Vẽ đường đi
            if 'iaco_path' in result and result['iaco_path'] is not None:
                self.plot_path(ax, result['iaco_path'],
                             color=self.color_iaco_path,
                             linewidth=2.0,
                             style='--',
                             alpha=0.7)
            
            if 'smooth_path' in result and result['smooth_path'] is not None:
                self.plot_path(ax, result['smooth_path'],
                             color=self.color_smooth_path,
                             linewidth=2.5,
                             style='-')
            
            # Vẽ start và goal
            if 'start_coord' in result and 'goal_coord' in result:
                self.plot_start_goal(ax, result['start_coord'], 
                                   result['goal_coord'], size=0.6)
            
            # Thiết lập axes
            ax.set_xlim(0, self.grid_map.x_max)
            ax.set_ylim(0, self.grid_map.y_max)
            ax.set_aspect('equal')
            ax.set_xlabel('x(m)', fontsize=10)
            ax.set_ylabel('y(m)', fontsize=10)
            ax.set_title(title, fontsize=12, fontweight='bold')
        
        plt.suptitle(suptitle, fontsize=16, fontweight='bold', y=0.98)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✓ Đã lưu: {save_path}")
        
        return fig, axes


if __name__ == "__main__":
    # Test PathPlotter
    print("Test PathPlotter Module\n")
    
    # Tạo grid map giống bài báo
    grid_map = GridMap(a=1.0, x_max=20.0, y_max=20.0)
    
    # Thêm chướng ngại vật (mô phỏng theo hình)
    obstacles = []
    
    # Chướng ngại vật mẫu
    obstacle_blocks = [
        (1, 0, 2, 2),   # (col, row, width, height)
        (5, 1, 3, 2),
        (15, 0, 2, 2),
        (18, 1, 1, 1),
        (0, 4, 2, 2),
        (6, 5, 4, 3),
        (14, 5, 2, 2),
        (18, 6, 1, 2),
    ]
    
    for col, row, width, height in obstacle_blocks:
        for i in range(row, min(row + height, grid_map.N_y)):
            for j in range(col, min(col + width, grid_map.N_x)):
                idx = grid_map.matrix_pos_to_grid_index(i, j)
                obstacles.append(idx)
    
    grid_map.add_obstacles_batch(obstacles)
    
    # Tạo plotter
    plotter = PathPlotter(grid_map, figsize=(8, 8))
    
    # Đường đi test
    iaco_path = np.array([
        [1, 19],
        [3, 17],
        [5, 15],
        [7, 13],
        [10, 10],
        [13, 7],
        [16, 4],
        [19, 1]
    ])
    
    smooth_path = np.array([
        [1, 19],
        [2.5, 17.5],
        [4.5, 15.5],
        [7, 13],
        [10, 10],
        [13, 7],
        [15.5, 4.5],
        [17.5, 2.5],
        [19, 1]
    ])
    
    # Vẽ IACO
    print("Vẽ IACO path...")
    plotter.plot_iaco_result(iaco_path, (1, 19), (19, 1),
                            title="IACO Path")
    
    # Vẽ B-spline
    print("Vẽ B-spline path...")
    plotter.plot_smooth_result(smooth_path, (1, 19), (19, 1),
                              title="B-spline Smoothed Path")
    
    plt.show()
    print("\n✓ Test visualization complete!")
