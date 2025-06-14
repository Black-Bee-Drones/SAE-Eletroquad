"""
Utility functions for distance estimation using pixel height.
"""
from hook.constants import (
    DISTANCE_POLY_A, 
    DISTANCE_POLY_B, 
    DISTANCE_POLY_C,
    DISTANCE_CALIBRATION_CONST
)


def estimate_distance_polynomial(height_px):
    """
    Estimate distance in cm from pixel height using polynomial fit.
    
    Args:
        height_px (float): Height of the object in pixels
        
    Returns:
        float: Estimated distance in centimeters
    """
    if height_px <= 0:
        return float('inf')  # Invalid height
        
    # Quadratic polynomial: distance = a*height^2 + b*height + c
    distance = DISTANCE_POLY_A * height_px**2 + DISTANCE_POLY_B * height_px + DISTANCE_POLY_C
    return max(0.0, distance)  # Ensure distance is not negative


def estimate_distance_linear(height_px):
    """
    Estimate distance in cm from pixel height using the original linear model.
    
    Args:
        height_px (float): Height of the object in pixels
        
    Returns:
        float: Estimated distance in centimeters
    """
    if height_px <= 0:
        return float('inf')  # Invalid height
        
    # Original linear model: distance = calibration_const / height
    return DISTANCE_CALIBRATION_CONST / height_px
