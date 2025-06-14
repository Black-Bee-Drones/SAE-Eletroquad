import matplotlib.pyplot as plt
import numpy as np
from hook.utils.distance_estimation import estimate_distance_polynomial, estimate_distance_linear


def main():
    # The measurement data we have
    measured_data = [
        (100, 21.52),  # Distance (cm), Height (px)
        (110, 20.32),
        (120, 19.9),
        (130, 19),
        (150, 18.1),
        (90, 22.2),
        (80, 24.4),
        (70, 25.6),
        (60, 28.2)
    ]

    # Create arrays for plotting
    measured_dist = np.array([point[0] for point in measured_data])
    measured_height = np.array([point[1] for point in measured_data])
    
    # Create a range of pixel heights to plot estimations
    pixel_heights = np.linspace(15, 40, 100)
    
    # Calculate estimated distances using both methods
    # Add some debug prints
    print("Calculating distances for heights:", pixel_heights[:5], "...")
    try:
        linear_distances = np.array([estimate_distance_linear(h) for h in pixel_heights])
        poly_distances = np.array([estimate_distance_polynomial(h) for h in pixel_heights])
        print("Calculation successful!")
    except Exception as e:
        print(f"Error calculating distances: {e}")
        import traceback
        traceback.print_exc()
    
    # Create plot
    plt.figure(figsize=(10, 8))
    
    # Plot the estimations
    plt.plot(pixel_heights, linear_distances, 'b-', label='Linear Model')
    plt.plot(pixel_heights, poly_distances, 'g-', label='Polynomial Model')
    
    # Plot the measured data points
    plt.scatter(measured_height, measured_dist, color='red', s=80, 
                label='Measured Data Points', zorder=5)
    
    # For each measured point, add a text annotation
    for i, (dist, height) in enumerate(zip(measured_dist, measured_height)):
        plt.annotate(f"{dist}cm, {height}px", 
                    (height, dist),
                    textcoords="offset points",
                    xytext=(0, 10),
                    ha='center')
    
    # Add labels and legend
    plt.xlabel('Pixel Height')
    plt.ylabel('Distance (cm)')
    plt.title('Distance Estimation Models Comparison')
    plt.grid(True)
    plt.legend()
    
    # Save the plot
    output_file = '/home/lucas/ros2_ws/src/SAE-Eletroquad/hook/distance_model_visualization.png'
    plt.savefig(output_file)
    plt.close()
    
    print(f"Visualization saved to {output_file}")
    
    # Also print a table of comparison for specific heights
    print("\nComparison Table:")
    print("-" * 70)
    print(f"{'Pixel Height':^15} | {'Linear Model (cm)':^20} | {'Polynomial Model (cm)':^20}")
    print("-" * 70)
    
    test_heights = [20, 21.42, 25, 30, 32, 33, 35]
    for h in test_heights:
        linear_est = estimate_distance_linear(h)
        poly_est = estimate_distance_polynomial(h)
        print(f"{h:^15.2f} | {linear_est:^20.2f} | {poly_est:^20.2f}")


if __name__ == "__main__":
    main()
