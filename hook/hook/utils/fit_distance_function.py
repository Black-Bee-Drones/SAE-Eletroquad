import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Your measured data points
# Format: Distance in cm, Height in pixels
data_points = [
    (100, 21.52),
    (110, 20.32),
    (120, 19.9),
    (130, 19),
    (150, 18.1),
    (90, 22.2),
    (80, 24.4),
    (70, 25.6),
    (60, 28.2),
]

# Separate into x (height in pixels) and y (distance in cm)
heights = np.array([point[1] for point in data_points])
distances = np.array([point[0] for point in data_points])

# Try different polynomial degrees
degrees = [1, 2, 3, 4]
fits = []
equations = []

plt.figure(figsize=(10, 8))
plt.scatter(heights, distances, color='red', label='Measured Data')

# Create a finer x-axis for plotting the fitted curves
height_range = np.linspace(min(heights) - 5, max(heights) + 5, 100)

for degree in degrees:
    # Fit polynomial of specified degree
    coeffs = np.polyfit(heights, distances, degree)
    poly = np.poly1d(coeffs)
    fits.append(poly)
    
    # Create a readable equation string
    eq = f"Distance = "
    for i, c in enumerate(coeffs):
        power = degree - i
        if power > 1:
            eq += f"{c:.4f} × height^{power} + "
        elif power == 1:
            eq += f"{c:.4f} × height + "
        else:
            eq += f"{c:.4f}"
    equations.append(eq)
    
    # Plot the fitted curve
    plt.plot(height_range, poly(height_range), label=f'Degree {degree} Fit')
    
    # Calculate R^2 (coefficient of determination)
    residuals = distances - poly(heights)
    ss_res = np.sum(residuals**2)
    ss_tot = np.sum((distances - np.mean(distances))**2)
    r_squared = 1 - (ss_res / ss_tot)
    print(f"Polynomial degree {degree}:")
    print(f"  Equation: {eq}")
    print(f"  Coefficients: {coeffs}")
    print(f"  R² value: {r_squared:.4f}")
    print()
    
    # Also test the inverse function (height to distance)
    inverse_coeffs = np.polyfit(distances, heights, degree)
    inverse_poly = np.poly1d(inverse_coeffs)
    inverse_eq = f"Height = "
    for i, c in enumerate(inverse_coeffs):
        power = degree - i
        if power > 1:
            inverse_eq += f"{c:.6f} × distance^{power} + "
        elif power == 1:
            inverse_eq += f"{c:.6f} × distance + "
        else:
            inverse_eq += f"{c:.6f}"
    print(f"  Inverse equation: {inverse_eq}")
    print()

# Function based on the original calibration constant
orig_constant = 1476.0  # cm*px
def original_model(height):
    return orig_constant / height

# Plot the original model
plt.plot(height_range, original_model(height_range), 'k--', label='Original Linear Model')

# Add labels and legend
plt.xlabel('Height (pixels)')
plt.ylabel('Distance (cm)')
plt.title('Distance vs. Pixel Height Relationship')
plt.grid(True)
plt.legend()

# Also create a graph showing prediction error
plt.figure(figsize=(10, 6))
for i, poly in enumerate(fits):
    errors = [(poly(h) - d) for h, d in zip(heights, distances)]
    plt.bar([i + x*0.2 for x in range(len(errors))], errors, width=0.2, 
            label=f'Degree {degrees[i]}')

plt.xlabel('Data Point')
plt.ylabel('Error (cm)')
plt.title('Prediction Error for Each Model')
plt.grid(True, axis='y')
plt.legend()

# Save the figures
plt.savefig('/home/lucas/ros2_ws/src/SAE-Eletroquad/hook/distance_model_comparison.png')
plt.close()

# Create a function we can use in our code
def get_distance_from_pixels(height_px):
    """
    Estimate distance in cm from pixel height using our best polynomial fit.
    """
    # Using the best fit (degree 2 polynomial)
    coeffs = fits[1].coefficients  # Using the quadratic fit (index 1)
    return np.polyval(coeffs, height_px)

# Test the function with some values
test_heights = [20, 25, 30, 35]
print("Testing the polynomial function:")
for h in test_heights:
    est_dist = get_distance_from_pixels(h)
    orig_dist = orig_constant / h
    print(f"Pixel height: {h}, Estimated distance: {est_dist:.2f} cm, Original model: {orig_dist:.2f} cm")

print("\nOriginal calibration constant:", orig_constant)
print("\nRecommended code to use in your project:")
print("""
# Distance estimation polynomial coefficients
# Function: distance = a * height^2 + b * height + c
DISTANCE_POLY_COEFFS = """ + str(fits[1].coefficients.tolist()))
