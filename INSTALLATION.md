# Hướng Dẫn Cài Đặt và Chạy

## Bước 1: Cài đặt Dependencies

```bash
pip install -r requirements.txt
```

Nếu gặp lỗi, cài từng package:

```bash
pip install numpy
pip install matplotlib
pip install scipy
pip install pandas
pip install jupyter
```

## Bước 2: Kiểm tra cài đặt

Chạy test để đảm bảo mọi thứ hoạt động:

```bash
# Test môi trường và IACO
python test_iaco.py

# Test B-spline và Ackermann
python test_smoothing.py
```

## Bước 3: Chạy Demo

### Option 1: Python Script

```bash
python demo_paper.py
```

Script này sẽ:
- Tạo bản đồ từ bài báo
- Chạy IACO để tìm đường đi
- Làm mượt bằng B-spline
- Lưu 4 hình vào thư mục `output/`

### Option 2: Jupyter Notebook (Khuyến nghị)

```bash
jupyter notebook demo_paper_results.ipynb
```

Hoặc mở bằng VS Code:
1. Mở file `demo_paper_results.ipynb`
2. Chọn Python kernel
3. Chạy từng cell (Shift + Enter)

## Các lỗi thường gặp

### Lỗi 1: ModuleNotFoundError

**Nguyên nhân:** Chưa cài đặt dependencies

**Giải pháp:**
```bash
pip install -r requirements.txt
```

### Lỗi 2: ImportError trong notebook

**Nguyên nhân:** Đường dẫn import không đúng

**Giải pháp:** Chạy cell đầu tiên để setup đường dẫn:
```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd().parent))
```

### Lỗi 3: "No module named 'src'"

**Nguyên nhân:** Chạy từ thư mục sai

**Giải pháp:** Đảm bảo chạy từ thư mục gốc:
```bash
cd "c:\Users\Admin\Desktop\Tinh toan tien hoa"
python demo_paper.py
```

### Lỗi 4: Matplotlib không hiển thị

**Nguyên nhân:** Backend không tương thích

**Giải pháp:** Trong notebook, thêm:
```python
%matplotlib inline
# hoặc
%matplotlib widget
```

## Cấu trúc thư mục sau khi chạy

```
Tinh toan tien hoa/
├── src/                    # Source code
├── output/                 # Kết quả (tự động tạo)
│   ├── fig_a_iaco.png
│   ├── fig_b_bspline.png
│   ├── fig_c_comparison.png
│   └── fig_d_curvature.png
├── demo_paper.py
├── demo_paper_results.ipynb
└── README.md
```

## Test riêng từng module

### Test GridMap
```python
from src.environment.grid_map import GridMap

grid_map = GridMap(a=1.0, x_max=20.0, y_max=20.0)
grid_map.print_info()
```

### Test IACO
```python
from src.aco.iaco import ImprovedACO

iaco = ImprovedACO(grid_map, start=1, goal=399, n_ants=10, n_iterations=20)
results = iaco.run(verbose=True)
```

### Test B-spline
```python
from src.ackermann.ackermann_model import AckermannModel
from src.path_smoothing.bspline import BSplineSmoothing

ackermann = AckermannModel(L=2.5, phi_max_deg=30.0)
smoother = BSplineSmoothing(ackermann, grid_size=1.0)
```

### Test Visualization
```python
from src.visualization.plotter import PathPlotter
import matplotlib.pyplot as plt

plotter = PathPlotter(grid_map)
fig, ax = plotter.create_figure(title="Test")
plt.show()
```

## Tùy chỉnh tham số

Để thay đổi tham số thuật toán, sửa trong `demo_paper.py` hoặc notebook:

```python
params = {
    'n_iterations': 100,    # Tăng để chính xác hơn
    'n_ants': 50,          # Tăng để tìm kiếm rộng hơn
    'alpha': 1.0,          # Hệ số pheromone
    'beta': 7.0,           # Hệ số heuristic
    'rho': 0.3,            # Bay hơi pheromone
    # ... các tham số khác
}

system = PathPlanningSystem(grid_map, start, goal, params=params)
```

## Liên hệ và hỗ trợ

Nếu gặp vấn đề:
1. Kiểm tra lại các bước cài đặt
2. Chạy test để xác định module lỗi
3. Xem log chi tiết của lỗi

---
**Cập nhật:** 16/10/2025
