"""
Demo Script - Recreate Paper Results
Tái tạo kết quả từ bài báo

Tác giả: GitHub Copilot
Ngày: 16/10/2025
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.environment.grid_map import GridMap
from src.integrated_system import PathPlanningSystem
from src.visualization.plotter import PathPlotter
from src.create_paper_map import create_paper_map, get_paper_start_goal


def main():
    """Chạy demo tái tạo kết quả bài báo"""
    
    print("\n" + "="*70)
    print(" "*15 + "TÁI TẠO KẾT QUẢ TỪ BÀI BÁO")
    print("="*70)
    print("Quy hoạch đường đi Robot Ackermann: IACO + B-spline")
    print("="*70 + "\n")
    
    # 1. Tạo bản đồ từ bài báo
    print("BƯỚC 1: Tạo bản đồ từ bài báo...")
    print("-" * 70)
    
    grid_map = create_paper_map(size=20.0, grid_size=1.0)
    start_node, goal_node, start_coord, goal_coord = get_paper_start_goal(grid_map)
    
    print(f"✓ Bản đồ: {grid_map.N_x}×{grid_map.N_y} = {grid_map.total_grids} grids")
    print(f"  Chướng ngại vật: {np.sum(grid_map.grid)} grids")
    print(f"  Start: node {start_node} tại {start_coord}")
    print(f"  Goal:  node {goal_node} tại {goal_coord}")
    
    # 2. Tạo hệ thống quy hoạch
    print("\n" + "="*70)
    print("BƯỚC 2: Khởi tạo hệ thống quy hoạch...")
    print("-" * 70)
    
    system = PathPlanningSystem(
        grid_map=grid_map,
        start=start_node,
        goal=goal_node,
        params=None  # Sử dụng tham số mặc định từ bài báo
    )
    
    # 3. Chạy quy hoạch đường đi
    print("\n" + "="*70)
    print("BƯỚC 3: Chạy quy hoạch đường đi...")
    print("-" * 70)
    
    results = system.plan(verbose=True)
    
    # 4. In tóm tắt
    system.print_summary()
    
    # 5. Vẽ kết quả
    if results['success']:
        print("\n" + "="*70)
        print("BƯỚC 4: Vẽ kết quả...")
        print("-" * 70)
        
        plotter = PathPlotter(grid_map, figsize=(10, 10))
        
        # Tạo thư mục output
        output_dir = Path('output')
        output_dir.mkdir(exist_ok=True)
        
        # (a) IACO path
        print("\n(a) Vẽ đường đi IACO...")
        fig1, ax1 = plotter.plot_iaco_result(
            path_coords=results['raw_path'],
            start_coord=start_coord,
            goal_coord=goal_coord,
            title="(a) IACO Optimal Path",
            save_path=str(output_dir / 'fig_a_iaco.png')
        )
        
        # (b) B-spline path
        print("(b) Vẽ đường đi B-spline...")
        fig2, ax2 = plotter.plot_smooth_result(
            smooth_coords=results['smooth_path'],
            start_coord=start_coord,
            goal_coord=goal_coord,
            curvatures=results['smoothing']['curvatures'],
            title="(b) B-spline Smoothed Path",
            save_path=str(output_dir / 'fig_b_bspline.png'),
            show_curvature=False
        )
        
        # (c) Comparison
        print("(c) Vẽ so sánh IACO vs B-spline...")
        fig3, ax3 = plotter.plot_comparison(
            iaco_coords=results['raw_path'],
            smooth_coords=results['smooth_path'],
            start_coord=start_coord,
            goal_coord=goal_coord,
            title="(c) Comparison: IACO vs B-spline",
            save_path=str(output_dir / 'fig_c_comparison.png')
        )
        
        # (d) Curvature analysis
        print("(d) Vẽ phân tích độ cong...")
        fig4, ax4 = plt.subplots(figsize=(12, 5))
        
        curvatures = results['smoothing']['curvatures']
        x_values = np.arange(len(curvatures))
        
        ax4.plot(x_values, curvatures, 'b-', linewidth=2, label='Độ cong ρ(u)')
        ax4.axhline(y=system.ackermann.rho_max, color='r', linestyle='--', 
                   linewidth=2, label=f'ρ_max = {system.ackermann.rho_max:.6f}')
        
        violations = curvatures > system.ackermann.rho_max
        if np.any(violations):
            ax4.fill_between(x_values, 0, curvatures, where=violations,
                           color='red', alpha=0.3, label='Vi phạm')
        
        ax4.set_xlabel('Vị trí dọc đường đi', fontsize=12)
        ax4.set_ylabel('Độ cong ρ (m⁻¹)', fontsize=12)
        ax4.set_title('(d) Phân tích độ cong dọc đường đi', fontsize=14, fontweight='bold')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'fig_d_curvature.png', dpi=300, bbox_inches='tight')
        print(f"  ✓ Đã lưu: {output_dir / 'fig_d_curvature.png'}")
        
        # Hiển thị tất cả
        print("\nHiển thị tất cả hình ảnh...")
        plt.show()
        
        print("\n" + "="*70)
        print("✓ HOÀN THÀNH TÁI TẠO KẾT QUẢ!")
        print("="*70)
        print(f"\nĐã lưu {4} hình vào thư mục: {output_dir.absolute()}")
        print("  - fig_a_iaco.png")
        print("  - fig_b_bspline.png")
        print("  - fig_c_comparison.png")
        print("  - fig_d_curvature.png")
        print("\n" + "="*70 + "\n")
        
    else:
        print("\n❌ Không tìm thấy đường đi!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠ Đã hủy bởi người dùng")
    except Exception as e:
        print(f"\n❌ Lỗi: {e}")
        import traceback
        traceback.print_exc()
