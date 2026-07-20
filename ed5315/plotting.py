"""
Matplotlib helpers for the end-of-run result plot each assignment's main.py
generates - map boundary, obstacles, goal, and the path(s) the robot took.
"""

import matplotlib
matplotlib.use('Agg')  # headless - no display needed to save a PNG
import matplotlib.pyplot as plt

from . import robot_params


def new_plot():
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    return fig, ax


def draw_boundary(ax, bounds=robot_params.arena_bounds):
    x_min, x_max, y_min, y_max = bounds
    ax.plot([x_min, x_max, x_max, x_min, x_min],
             [y_min, y_min, y_max, y_max, y_min],
             color='black', linewidth=1.5, label='Boundary')
    ax.set_xlim(x_min - 1, x_max + 1)
    ax.set_ylim(y_min - 1, y_max + 1)


def draw_obstacles(ax, positions, color='red', marker='o', label='Obstacles', radius=None):
    if not positions:
        return
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    ax.scatter(xs, ys, c=color, marker=marker, s=100, label=label, zorder=3, edgecolors='black')
    if radius:
        for x, y in zip(xs, ys):
            ax.add_patch(plt.Circle((x, y), radius, color=color, alpha=0.15, zorder=1))


def draw_goal(ax, position, color='limegreen', marker='*', label='Goal'):
    ax.scatter([position[0]], [position[1]], c=color, marker=marker, s=350,
               label=label, zorder=4, edgecolors='black')


def draw_path(ax, xy_list, color='blue', label='Path', linestyle='-'):
    if not xy_list:
        return
    xs = [p[0] for p in xy_list]
    ys = [p[1] for p in xy_list]
    ax.plot(xs, ys, color=color, linestyle=linestyle, linewidth=2, label=label, zorder=2)
    ax.scatter([xs[0]], [ys[0]], c=color, marker='s', s=60, zorder=5)  # start marker


def finish_plot(ax, title, save_path):
    ax.legend(loc='best', fontsize=9)
    ax.set_title(title)
    ax.figure.tight_layout()
    ax.figure.savefig(save_path, dpi=150)
    plt.close(ax.figure)
    print(f'Saved result plot to {save_path}')
