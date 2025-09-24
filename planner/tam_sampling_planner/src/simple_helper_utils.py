#!/usr/bin/env python3
"""
Simple helper utilities for TAM Sampling Planner
Provides fallback implementations when planning_common is not available
"""

import numpy as np
import rospy


def create_trim_mask(array, target_length):
    """
    Create a boolean mask to trim array to target length.

    Args:
        array: Input array to trim
        target_length: Desired length

    Returns:
        np.ndarray: Boolean mask of length len(array)
    """
    if len(array) <= target_length:
        return np.ones(len(array), dtype=bool)

    # Create evenly spaced indices
    indices = np.linspace(0, len(array) - 1, target_length, dtype=int)
    mask = np.zeros(len(array), dtype=bool)
    mask[indices] = True

    return mask


def create_trim_mask_2d(array_2d, target_length):
    """
    Create a boolean mask to trim 2D array along first dimension.

    Args:
        array_2d: Input 2D array to trim
        target_length: Desired length along first dimension

    Returns:
        np.ndarray: Boolean mask of length array_2d.shape[0]
    """
    if array_2d.shape[0] <= target_length:
        return np.ones(array_2d.shape[0], dtype=bool)

    # Create evenly spaced indices
    indices = np.linspace(0, array_2d.shape[0] - 1, target_length, dtype=int)
    mask = np.zeros(array_2d.shape[0], dtype=bool)
    mask[indices] = True

    return mask


def find_nearest_s_and_idx(s_array, target_s, track_handler=None):
    """
    Find the nearest arc length value and its index in the array.

    Args:
        s_array: Array of arc length values
        target_s: Target arc length value to find
        track_handler: Track handler (optional, for periodic tracks)

    Returns:
        tuple: (nearest_s_value, nearest_index)
    """
    s_array = np.asarray(s_array)

    # Handle periodic tracks (wrap around)
    if track_handler is not None and hasattr(track_handler, 'get_track_length'):
        track_length = track_handler.get_track_length()
        if track_length > 0:
            # Normalize target_s to track length
            target_s = target_s % track_length

    # Find closest index
    differences = np.abs(s_array - target_s)
    nearest_idx = np.argmin(differences)
    nearest_s = s_array[nearest_idx]

    return nearest_s, nearest_idx


def interpolate_with_period(x, xp, fp, period=None):
    """
    Interpolate with periodic boundary conditions.

    Args:
        x: Points at which to evaluate interpolated values
        xp: Known data points (must be sorted)
        fp: Known function values at xp
        period: Period for periodic interpolation (optional)

    Returns:
        np.ndarray: Interpolated values
    """
    if period is None:
        return np.interp(x, xp, fp)

    # Handle periodic interpolation
    x = np.asarray(x)
    x_mod = x % period

    # Extend xp and fp for periodic interpolation
    xp_extended = np.concatenate([xp - period, xp, xp + period])
    fp_extended = np.concatenate([fp, fp, fp])

    return np.interp(x_mod, xp_extended, fp_extended)


def validate_array_monotonic(array, name="array"):
    """
    Validate that an array is monotonically increasing.

    Args:
        array: Array to validate
        name: Name for error messages

    Returns:
        bool: True if monotonic, False otherwise
    """
    if len(array) < 2:
        return True

    is_monotonic = np.all(np.diff(array) >= 0)
    if not is_monotonic:
        rospy.logwarn(f"{name} is not monotonically increasing")

    return is_monotonic


def safe_divide(numerator, denominator, default_value=0.0):
    """
    Perform safe division with fallback for zero denominators.

    Args:
        numerator: Numerator values
        denominator: Denominator values  
        default_value: Value to use when denominator is zero

    Returns:
        np.ndarray: Result of division with safe handling
    """
    numerator = np.asarray(numerator)
    denominator = np.asarray(denominator)

    # Create result array
    result = np.full_like(numerator, default_value, dtype=float)

    # Only divide where denominator is non-zero
    nonzero_mask = np.abs(denominator) > 1e-10
    result[nonzero_mask] = numerator[nonzero_mask] / denominator[nonzero_mask]

    return result


def smooth_array(array, window_size=5):
    """
    Apply simple moving average smoothing to an array.

    Args:
        array: Input array to smooth
        window_size: Size of smoothing window (must be odd)

    Returns:
        np.ndarray: Smoothed array
    """
    if len(array) < window_size:
        return array

    # Ensure window size is odd
    if window_size % 2 == 0:
        window_size += 1

    # Apply moving average
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(array, kernel, mode='same')

    return smoothed


# Test functions
if __name__ == "__main__":
    # Test the utility functions
    print("Testing helper utilities...")

    # Test trim mask
    test_array = np.arange(20)
    mask = create_trim_mask(test_array, 10)
    print(f"Trim mask test: {len(test_array)} -> {np.sum(mask)} elements")

    # Test find nearest
    s_array = np.array([0, 5, 10, 15, 20])
    nearest_s, nearest_idx = find_nearest_s_and_idx(s_array, 12)
    print(
        f"Find nearest test: target=12, found s={nearest_s}, idx={nearest_idx}")

    # Test smoothing
    noisy_array = np.random.randn(20)
    smoothed = smooth_array(noisy_array, 5)
    print(
        f"Smoothing test: original std={np.std(noisy_array):.3f}, smoothed std={np.std(smoothed):.3f}")

    print("All tests completed!")
