# Quy hoạch đường đi mượt mà cho Robot Ackermann

Dự án mô phỏng quy hoạch đường đi cho robot di động Ackermann sử dụng:
- **Thuật toán kiến cải tiến (IACO-TAC)** với hàm heuristic cải tiến
- **Đường cong B-spline bậc 3** để làm mượt đường đi
- **Ràng buộc động học Ackermann** cho robot thực tế

## 🎯 Tính năng chính

### 1. Môi trường (Environment)
- Lưới grid map với chướng ngại vật
- Chuyển đổi tọa độ grid ↔ tọa độ thực
- Quản lý các node láng giềng (4/8 hướng)

### 2. Thuật toán IACO-TAC
- **Hàm heuristic cải tiến**: Kết hợp khoảng cách + góc quay
- **Xác suất chuyển trạng thái**: P^k_ij = [τ_ij]^α × [η_ij]^β
- **Đánh giá đa mục tiêu**: S(p) = k_L×L(p) + k_E×E(p)
- **Cập nhật pheromone hai tầng**: Bay hơi + thưởng/phạt + elite
- **Giới hạn MMAS**: τ_min < τ < τ_max

### 3. Làm mượt B-spline
- **Đường cong B-spline bậc 3** với 4 hàm cơ sở
- **Tính độ cong**: ρ(u) = |P''×P'| / |P'|³
- **Tối ưu điểm kiểm soát**: Thêm điểm dựa trên góc quay
- **Kiểm tra ràng buộc**: ρ(u) ≤ ρ_max

### 4. Mô hình Ackermann
- Góc lái tối đa: |φ| ≤ φ_max
- Bán kính quay: R = L/tan(φ)
- Độ cong tối đa: ρ_max = tan(φ_max)/L

## 📁 Cấu trúc dự án

```
├── src/
│   ├── environment/           # Mô hình hóa môi trường
│   │   ├── __init__.py
│   │   └── grid_map.py       # Lưới grid map
│   │
│   ├── aco/                  # Thuật toán kiến cải tiến
│   │   ├── __init__.py
│   │   ├── heuristic.py      # Hàm heuristic + đánh giá
│   │   ├── ant.py            # Lớp Ant
│   │   ├── pheromone.py      # Ma trận pheromone
│   │   ├── iaco.py           # Thuật toán IACO
│   │   └── README.md
│   │
│   ├── ackermann/            # Mô hình robot Ackermann
│   │   ├── __init__.py
│   │   └── ackermann_model.py
│   │
│   ├── visualization/         # Visualization và vẽ đồ thị
│   │   ├── __init__.py
│   │   └── plotter.py        # PathPlotter
│   │
│   ├── create_paper_map.py   # Tạo bản đồ từ bài báo
│   └── integrated_system.py  # Hệ thống tích hợp
│
├── demo_paper.py             # Demo tái tạo kết quả
├── demo_paper_results.ipynb  # Jupyter Notebook demo
├── test_iaco.py              # Test IACO
├── test_smoothing.py         # Test B-spline
├── requirements.txt          # Dependencies
└── README.md
│   │   └── ackermann_model.py
│   │
│   ├── path_smoothing/       # Làm mượt B-spline
│   │   ├── __init__.py
│   │   ├── bspline.py        # Đường cong B-spline
│   │   ├── control_point_optimizer.py
│   │   └── README.md
│   │
│   └── integrated_system.py  # Hệ thống tích hợp
│
├── test_iaco.py              # Test thuật toán IACO
├── test_smoothing.py         # Test làm mượt
├── requirements.txt          # Dependencies
└── README.md

```

## 🚀 Cài đặt

### 1. Clone repository hoặc tải về

### 2. Cài đặt dependencies:

```bash
pip install -r requirements.txt
```

Dependencies:
- numpy >= 1.24.0
- matplotlib >= 3.7.0
- scipy >= 1.10.0
- pandas >= 2.0.0

## 💻 Sử dụng

### Quick Start - Tái tạo kết quả từ bài báo:

#### Option 1: Chạy script Python
```bash
python demo_paper.py
```

#### Option 2: Jupyter Notebook (Khuyến nghị)
```bash
jupyter notebook demo_paper_results.ipynb
```

Notebook bao gồm:
- ✓ Tạo bản đồ từ bài báo
- ✓ Chạy IACO tìm đường đi
- ✓ Làm mượt B-spline
- ✓ 4 hình visualization
- ✓ Phân tích độ cong
- ✓ Biểu đồ hội tụ

### Ví dụ code cơ bản:

```python
from src.environment.grid_map import GridMap
from src.integrated_system import PathPlanningSystem

# 1. Tạo môi trường
grid_map = GridMap(a=1.0, x_max=50.0, y_max=50.0)
grid_map.add_obstacles_batch([...])

# 2. Định nghĩa start và goal
start = grid_map.coord_to_grid_index(2, 2)
goal = grid_map.coord_to_grid_index(48, 48)

# 3. Tạo hệ thống quy hoạch
system = PathPlanningSystem(
    grid_map=grid_map,
    start=start,
    goal=goal,
    params=None  # Sử dụng tham số mặc định
)

# 4. Chạy quy hoạch
results = system.plan(verbose=True)

# 5. Xem kết quả
system.print_summary()
```

### Chạy tests:

```bash
# Test IACO
python test_iaco.py

# Test B-spline smoothing
python test_smoothing.py
```

## 📊 Tham số đề xuất

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| K (iterations) | 100 | Số lần lặp |
| M (ants) | 50 | Số kiến |
| α (alpha) | 1.0 | Hệ số pheromone |
| β (beta) | 7.0 | Hệ số heuristic |
| ρ (rho) | 0.3 | Hệ số bay hơi |
| Q | 100.0 | Hằng số pheromone |
| λ₁ | 0.6 | Trọng số khoảng cách |
| λ₂ | 0.4 | Trọng số góc quay |
| k_L | 0.7 | Trọng số độ dài |
| k_E | 0.3 | Trọng số năng lượng |
| L | 2.5 m | Wheelbase robot |
| φ_max | 30° | Góc lái tối đa |

## 📚 Chi tiết thuật toán

### IACO-TAC (Improved ACO with Two-tier Ant Colony)

**Công thức chính:**

1. **Heuristic cải tiến**:
   ```
   η_ij = 1 / (λ₁×d_jg + λ₂×f_cost)
   ```

2. **Xác suất chuyển trạng thái**:
   ```
   P^k_ij = [τ_ij]^α × [η_ij]^β / Σ[τ_ij]^α × [η_ij]^β
   ```

3. **Hàm mục tiêu**:
   ```
   min S(p) = k_L×L(p) + k_E×E(p)
   ```

4. **Cập nhật pheromone hai tầng**:
   ```
   τ_ij(t+1) = (1-ρ)×τ_ij(t) + Δτ_ij(t) + Q/S_Best
   ```

### B-spline bậc 3

**Công thức B-spline**:
```
P(u) = Σ P_i × B_{i,3}(u)
```

**Độ cong**:
```
ρ(u) = |P''(u) × P'(u)| / |P'(u)|³
```

**Ràng buộc**:
```
ρ(u) ≤ ρ_max = tan(φ_max)/L
```

## 📖 Tài liệu

Xem chi tiết trong các file README:
- [ACO Module](src/aco/README.md)
- [Path Smoothing](src/path_smoothing/README.md)

## 🔬 Kết quả mong đợi

- ✅ Tìm đường đi tối ưu tránh chướng ngại vật
- ✅ Đường đi mượt mà, liên tục
- ✅ Thỏa mãn ràng buộc góc lái robot Ackermann
- ✅ Tối ưu độ dài và năng lượng tiêu tốn
- ✅ Độ cong không vượt quá khả năng robot

## 🤝 Đóng góp

Dự án này được phát triển cho mục đích học tập và nghiên cứu.

---
**Tác giả:** GitHub Copilot  
**Ngày tạo:** 16/10/2025  
**Phiên bản:** 1.0.0
