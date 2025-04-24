# Bézier Curve Generator

This Python script provides a simple way to compute and visualize Bézier curves using an arbitrary number of control points. It includes functions for calculating the curve and plotting both the curve and the control polygon.

## Features

- Uses **Bernstein polynomials** to compute Bézier curves
- Supports **any number of control points**
- Generates **smooth 2D trajectories**
- Visualizes curves and control polygons using `matplotlib`

## Dependencies

Make sure you have the following Python libraries installed:

```bash
pip install numpy matplotlib scipy
```

## Usage

### 1. Define your control points

Control points must be provided as a list of `[x, y]` pairs. For example:

```python
control_points = np.array([
    [0, 0],
    [1, 2],
    [3, 3],
    [4, 0]
])
```
or
```python
control_points = np.array([
    [0, 0],
    [2, 0],
    [2, 2],
    [0, 2],
    [0, 0]
])
```

### 2. Generate and plot the Bézier curve

```python
from BezierCurve import bezier_curve, plot_trajectory

curve, x_vals, y_vals = bezier_curve(control_points, num_samples=100)
plot_trajectory(control_points, curve)
```
- `num_samples`: The number of points to sample along the Bézier curve. Higher values produce a smoother curve (default is 100).

## Functions

### `bernstein_poly(n, i, t)`
Computes the Bernstein polynomial of `n`, `i`, and `comb(n, i)` at parameter `t`.
- `n`: Total number of control points minus one
- `i`: The index of the control point
- `t`: Curve parameter (ranges from 0 to 1)
- `comb(n, i)`: Binomial coefficient

### `bezier_curve(control_points, num_samples=100)`
Generates a Bézier curve from the given control points. Returns the full curve and separate x/y values.

### `plot_trajectory(control_points, curve)`
Plots the Bézier curve along with the control points and lines connecting them.