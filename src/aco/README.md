# PHẦN 2: THUẬT TOÁN KIẾN CẢI TIẾN (IACO-TAC)

## Tổng quan

Phần này implement thuật toán kiến cải tiến với các tính năng:

### 1. Hàm Heuristic Cải tiến (`heuristic.py`)

**Các thành phần:**
- **Khoảng cách Euclidean** (Công thức 11)
- **Góc quay robot** (Công thức 12-14)
- **Hệ số phạt góc lái** (Công thức 15)
- **Hàm heuristic tổng hợp** (Công thức 16)

**Công thức chính:**
```
η_ij = 1 / (λ₁ × d_jg + λ₂ × f_cost)
```

### 2. Xác Suất Chuyển Trạng Thái (`ant.py`)

**Công thức (2):**
```
P^k_ij = [τ_ij(t)]^α × [η_ij(t)]^β / Σ[τ_ij(t)]^α × [η_ij(t)]^β
```

**Tham số:**
- α ∈ [1, 4]: Hệ số pheromone
- β ∈ [4, 7]: Hệ số kỳ vọng

### 3. Đánh Giá Đa Mục Tiêu (`heuristic.py`)

**Độ dài đường đi (Công thức 17-18):**
```
L(p) = Σ d(p_i, p_{i+1})
```

**Năng lượng tiêu tốn (Công thức 19-21):**
```
E(p) = Σ P × |θ_{i+1} - θ_i|
```

**Hàm mục tiêu (Công thức 22-23):**
```
min S(p) = k_L × L(p) + k_E × E(p)
```

### 4. Cập Nhật Pheromone Hai Tầng (`pheromone.py`)

**Cập nhật tổng thể (Công thức 24):**
```
τ_ij(t+1) = (1 - ρ) × τ_ij(t) + Δτ_ij(t) + Q/S_Best
```

**Cơ chế thưởng phạt (Công thức 26-27):**
```
τ^k_ij(t) = w × Q/S_k(t)

w = { 1  nếu S ≤ (S_best + S_worst)/2
    {-1  nếu S > (S_best + S_worst)/2
```

**Giới hạn MMAS (Công thức 28):**
```
τ_min < τ < τ_max
```

## Cấu trúc Files

```
src/aco/
├── __init__.py           # Package initialization
├── heuristic.py          # Hàm heuristic và đánh giá đường đi
├── ant.py                # Lớp Ant (kiến)
├── pheromone.py          # Ma trận pheromone
└── iaco.py               # Thuật toán IACO chính
```

## Sử dụng

```python
from src.environment.grid_map import GridMap
from src.aco.iaco import ImprovedACO

# Tạo grid map
grid_map = GridMap(a=1.0, x_max=100.0, y_max=100.0)

# Thêm chướng ngại vật
grid_map.add_obstacles_batch([...])

# Chạy IACO
iaco = ImprovedACO(
    grid_map=grid_map,
    start=1,
    goal=9999,
    n_ants=30,
    n_iterations=100,
    alpha=2.0,
    beta=5.0,
    rho=0.3,
    lambda1=0.6,
    lambda2=0.4,
    k_L=0.7,
    k_E=0.3
)

results = iaco.run(verbose=True)
iaco.print_results()
```

## Tham số Khuyến nghị

| Tham số | Ký hiệu | Khuyến nghị | Mô tả |
|---------|---------|-------------|-------|
| Hệ số pheromone | α | [1, 4] | Ảnh hưởng của pheromone |
| Hệ số heuristic | β | [4, 7] | Ảnh hưởng của heuristic |
| Bay hơi | ρ | [0.2, 0.5] | Tốc độ bay hơi pheromone |
| Hệ số phạt góc | P | [2, 4] | Phạt khi quay góc lớn |
| Trọng số khoảng cách | λ₁ | 0.5-0.7 | Ưu tiên đường ngắn |
| Trọng số góc quay | λ₂ | 0.3-0.5 | Ưu tiên đường mượt |
| Trọng số độ dài | k_L | 0.6-0.8 | Quan trọng độ dài |
| Trọng số năng lượng | k_E | 0.2-0.4 | Quan trọng năng lượng |

## Test

Chạy test:
```bash
python test_iaco.py
```

---
**Ngày:** 16/10/2025
