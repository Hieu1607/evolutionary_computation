# PHẦN 3: LÀM MƯỢT ĐƯỜNG ĐI BẰNG B-SPLINE

## Tổng quan

Module làm mượt đường đi bằng B-spline với ràng buộc động học robot Ackermann.

## 1. Mô Hình Robot Ackermann (`ackermann_model.py`)

### Ràng buộc động học:

**Góc lái tối đa (Công thức 29):**
```
|φ| ≤ φ_max
```

**Bán kính quay (Công thức 30):**
```
R = L / tan(φ)
```

**Độ cong tối đa (Công thức 31):**
```
ρ_max = 1/R_min = tan(φ_max)/L
```

### Sử dụng:
```python
from src.ackermann.ackermann_model import AckermannModel

# Tạo mô hình
model = AckermannModel(L=2.5, phi_max_deg=30.0)

# Kiểm tra góc lái
valid = model.validate_steering_angle(phi)

# Kiểm tra độ cong
valid = model.validate_curvature(rho)
```

## 2. Đường Cong B-Spline (`bspline.py`)

### Hàm cơ sở B-spline bậc 3 (Công thức 33):

```
B_{0,3}(u) = (1/6) × (-u³ + 3u² - 3u + 1)
B_{1,3}(u) = (1/6) × (3u³ - 6u² + 4)
B_{2,3}(u) = (1/6) × (-3u³ + 3u² + 3u + 1)
B_{3,3}(u) = (1/6) × u³
```

### Công thức B-spline (Công thức 32):
```
P(u) = Σ(i=0 đến n) P_i × B_{i,3}(u)
```

### Độ cong (Công thức 34):
```
ρ(u) = |P''(u) × P'(u)| / |P'(u)|³
```

### Ràng buộc độ cong (Công thức 35):
```
ρ(u) ≤ ρ_max
```

### Sử dụng:
```python
from src.path_smoothing.bspline import BSpline

bspline = BSpline()

# Tạo đường cong
curve, curvatures = bspline.generate_curve(control_points, n_samples=100)

# Kiểm tra ràng buộc
is_valid, violated = bspline.check_curvature_constraint(
    control_points, ackermann_model
)
```

## 3. Tối Ưu Điểm Kiểm Soát (`control_point_optimizer.py`)

### Chiến lược thêm điểm:

**Dựa trên góc quay:**
- Góc > 90°: Thêm điểm tại 1/3 khoảng cách
- Góc ≤ 90°: Thêm điểm tại 1/5 khoảng cách

**Sửa vi phạm độ cong:**
- Tạo tam giác cân với offset = 1/2 grid size
- Thay thế node vi phạm bằng 2 node mới
- Chuyển góc tù → 2 góc nhọn

### Sử dụng:
```python
from src.path_smoothing.control_point_optimizer import BSplineSmoothing

smoother = BSplineSmoothing(ackermann_model, grid_size=1.0)

# Làm mượt đường đi
result = smoother.smooth_path(
    path_coords,
    n_samples=100,
    optimize=True
)
```

## 4. Hệ Thống Tích Hợp (`integrated_system.py`)

Kết hợp IACO + B-spline + Ackermann constraints.

### Quy trình:

1. **IACO**: Tìm đường đi tối ưu
2. **B-spline**: Làm mượt đường đi
3. **Validation**: Kiểm tra ràng buộc Ackermann

### Sử dụng:
```python
from src.integrated_system import PathPlanningSystem

system = PathPlanningSystem(
    grid_map=grid_map,
    start=start_node,
    goal=goal_node,
    params=None  # Sử dụng tham số mặc định
)

results = system.plan(verbose=True)
system.print_summary()
```

## 5. Tham Số Đề Xuất (Bảng 1)

| Tham số | Ký hiệu | Giá trị | Mô tả |
|---------|---------|---------|-------|
| Số lần lặp | K | 100 | Iterations tối đa |
| Số kiến | M | 50 | Số kiến trong colony |
| Hệ số pheromone | α | 1 | Ảnh hưởng pheromone |
| Hệ số bay hơi | ρ | 0.3 | Tốc độ bay hơi |
| Hệ số kỳ vọng | β | 7 | Ảnh hưởng heuristic |
| Hằng số | Q | 100 | Hằng số pheromone |
| Wheelbase | L | 2.5 m | Chiều dài trục xe |
| Góc lái max | φ_max | 30° | Góc lái tối đa |

## Cấu trúc Files

```
src/
├── ackermann/
│   ├── __init__.py
│   └── ackermann_model.py      # Mô hình động học Ackermann
├── path_smoothing/
│   ├── __init__.py
│   ├── bspline.py              # Đường cong B-spline
│   └── control_point_optimizer.py  # Tối ưu điểm kiểm soát
└── integrated_system.py         # Hệ thống tích hợp
```

## Test

```bash
python test_smoothing.py
```

## Ví dụ Đầy Đủ

```python
from src.environment.grid_map import GridMap
from src.integrated_system import PathPlanningSystem

# Tạo môi trường
grid_map = GridMap(a=1.0, x_max=50.0, y_max=50.0)
grid_map.add_obstacles_batch([...])

# Quy hoạch đường đi
system = PathPlanningSystem(
    grid_map=grid_map,
    start=1,
    goal=2499
)

results = system.plan(verbose=True)

# Kết quả
if results['success']:
    print(f"Đường đi thô: {len(results['raw_path'])} điểm")
    print(f"Đường đi mượt: {len(results['smooth_path'])} điểm")
    print(f"Độ cong max: {results['smoothing']['max_curvature']:.6f}")
    print(f"Ackermann OK: {results['ackermann_valid']}")
```

---
**Ngày:** 16/10/2025
