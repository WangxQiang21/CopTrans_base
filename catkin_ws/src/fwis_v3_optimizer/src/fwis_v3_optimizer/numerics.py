"""Small compatibility boundary for ROS NumPy 1.x and learning NumPy 2.x."""
import numpy as np

trapezoid = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
