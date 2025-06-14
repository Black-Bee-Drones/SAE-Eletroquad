import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from sklearn.linear_model import HuberRegressor
import warnings

warnings.filterwarnings("ignore")


class DistanceModelAnalyzer:
    def __init__(self, data_points):
        self.data_points = sorted(data_points, key=lambda x: x[0])
        self.distances = np.array([point[0] for point in self.data_points])
        self.heights = np.array([point[1] for point in self.data_points])
        self.models = {}
        self.metrics = {}

    def polynomial_model(self, degree):
        """Fit polynomial model of given degree"""
        coeffs = np.polyfit(self.heights, self.distances, degree)
        return np.poly1d(coeffs), coeffs

    def inverse_linear_model(self, height, k):
        """Original inverse linear model"""
        return k / height

    def inverse_power_model(self, height, k, p):
        """Generalized inverse power model"""
        return k / (height**p)

    def exponential_model(self, height, a, b, c):
        """Exponential decay model"""
        return a * np.exp(-b * height) + c

    def rational_model(self, height, a, b, c, d):
        """Rational function model"""
        return (a * height + b) / (c * height + d)

    def logarithmic_model(self, height, a, b):
        """Logarithmic model"""
        return a * np.log(height) + b

    def fit_all_models(self):
        """Fit all available models and store results"""

        # Polynomial models (degrees 1-4)
        for degree in [1, 2, 3, 4]:
            try:
                poly, coeffs = self.polynomial_model(degree)
                predictions = poly(self.heights)
                self.models[f"poly_{degree}"] = {
                    "function": poly,
                    "coeffs": coeffs,
                    "predictions": predictions,
                    "name": f"Polynomial Degree {degree}",
                }
            except Exception as e:
                print(f"Failed to fit polynomial degree {degree}: {e}")

        # Inverse power model
        try:
            popt, _ = curve_fit(
                self.inverse_power_model,
                self.heights,
                self.distances,
                p0=[2150, 1],
                bounds=([0, 0.1], [10000, 3]),
            )
            predictions = self.inverse_power_model(self.heights, *popt)
            self.models["inverse_power"] = {
                "function": lambda h: self.inverse_power_model(h, *popt),
                "coeffs": popt,
                "predictions": predictions,
                "name": f"Inverse Power (k={popt[0]:.1f}, p={popt[1]:.2f})",
            }
        except Exception as e:
            print(f"Failed to fit inverse power model: {e}")

        # Exponential model
        try:
            popt, _ = curve_fit(
                self.exponential_model,
                self.heights,
                self.distances,
                p0=[100, 0.1, 10],
                maxfev=2000,
            )
            predictions = self.exponential_model(self.heights, *popt)
            self.models["exponential"] = {
                "function": lambda h: self.exponential_model(h, *popt),
                "coeffs": popt,
                "predictions": predictions,
                "name": f"Exponential Decay",
            }
        except Exception as e:
            print(f"Failed to fit exponential model: {e}")

        # Logarithmic model
        try:
            popt, _ = curve_fit(self.logarithmic_model, self.heights, self.distances)
            predictions = self.logarithmic_model(self.heights, *popt)
            self.models["logarithmic"] = {
                "function": lambda h: self.logarithmic_model(h, *popt),
                "coeffs": popt,
                "predictions": predictions,
                "name": f"Logarithmic",
            }
        except Exception as e:
            print(f"Failed to fit logarithmic model: {e}")

        # Original linear model
        orig_k = 2150.0
        predictions = orig_k / self.heights
        self.models["original"] = {
            "function": lambda h: orig_k / h,
            "coeffs": [orig_k],
            "predictions": predictions,
            "name": "Original Linear (k=2150)",
        }

        # Robust regression (polynomial degree 2 with Huber loss)
        try:
            # Create polynomial features
            X = np.column_stack(
                [self.heights**2, self.heights, np.ones(len(self.heights))]
            )
            huber = HuberRegressor(epsilon=1.5, max_iter=200)
            huber.fit(X, self.distances)
            predictions = huber.predict(X)
            self.models["robust_poly2"] = {
                "function": lambda h: huber.predict(
                    np.column_stack(
                        [h**2, h, np.ones(len(h) if hasattr(h, "__len__") else 1)]
                    )
                ),
                "coeffs": huber.coef_,
                "predictions": predictions,
                "name": "Robust Polynomial (Degree 2)",
            }
        except Exception as e:
            print(f"Failed to fit robust model: {e}")

    def calculate_metrics(self):
        """Calculate performance metrics for all models"""
        for name, model in self.models.items():
            predictions = model["predictions"]

            mse = mean_squared_error(self.distances, predictions)
            rmse = np.sqrt(mse)
            mae = mean_absolute_error(self.distances, predictions)
            r2 = r2_score(self.distances, predictions)

            # Calculate AIC (Akaike Information Criterion)
            n = len(self.distances)
            k = len(model["coeffs"])  # number of parameters
            aic = n * np.log(mse) + 2 * k

            # Calculate relative error
            rel_errors = np.abs((predictions - self.distances) / self.distances * 100)
            mean_rel_error = np.mean(rel_errors)
            max_rel_error = np.max(rel_errors)

            self.metrics[name] = {
                "mse": mse,
                "rmse": rmse,
                "mae": mae,
                "r2": r2,
                "aic": aic,
                "mean_rel_error": mean_rel_error,
                "max_rel_error": max_rel_error,
                "n_params": k,
            }

    def plot_models_comparison(self, save_path=None):
        """Create comprehensive comparison plots"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

        # Generate smooth curve for plotting
        height_range = np.linspace(min(self.heights) - 3, max(self.heights) + 3, 200)

        colors = plt.cm.tab10(np.linspace(0, 1, len(self.models)))

        # Plot 1: Model fits
        ax1.scatter(
            self.heights,
            self.distances,
            color="red",
            s=100,
            label="Measured Points",
            zorder=10,
            edgecolor="black",
        )

        for i, (name, model) in enumerate(self.models.items()):
            try:
                if name == "robust_poly2":
                    # Handle robust model separately
                    X_range = np.column_stack(
                        [height_range**2, height_range, np.ones(len(height_range))]
                    )
                    y_range = model["function"](height_range)
                elif name == "exponential":
                    # Handle exponential model with coefficients
                    y_range = self.exponential_model(height_range, *model["coeffs"])
                else:
                    y_range = model["function"](height_range)

                ax1.plot(
                    height_range,
                    y_range,
                    color=colors[i],
                    label=f"{model['name']} (R²={self.metrics[name]['r2']:.3f})",
                    linewidth=2,
                )
            except Exception as e:
                print(f"Error plotting {name}: {e}")

        ax1.set_xlabel("Height (pixels)")
        ax1.set_ylabel("Distance (cm)")
        ax1.set_title("Model Fits Comparison")
        ax1.grid(True, alpha=0.3)
        ax1.legend(bbox_to_anchor=(1.05, 1), loc="upper left")

        # Plot 2: Residuals
        for i, (name, model) in enumerate(self.models.items()):
            residuals = model["predictions"] - self.distances
            ax2.plot(
                self.heights,
                residuals,
                "o-",
                color=colors[i],
                label=f"{model['name']} (RMSE={self.metrics[name]['rmse']:.2f})",
            )

        ax2.axhline(y=0, color="red", linestyle="--", alpha=0.7)
        ax2.set_xlabel("Height (pixels)")
        ax2.set_ylabel("Residuals (Predicted - Actual)")
        ax2.set_title("Residual Analysis")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Plot 3: Metrics comparison
        metrics_names = ["rmse", "mae", "mean_rel_error"]
        metrics_labels = ["RMSE (cm)", "MAE (cm)", "Mean Rel. Error (%)"]

        for j, (metric, label) in enumerate(zip(metrics_names, metrics_labels)):
            ax3_sub = ax3 if j == 0 else ax3.twinx()

            values = [self.metrics[name][metric] for name in self.models.keys()]
            model_names = [
                self.models[name]["name"][:15] for name in self.models.keys()
            ]

            bars = ax3_sub.bar(
                np.arange(len(values)) + j * 0.25,
                values,
                width=0.25,
                alpha=0.7,
                label=label,
            )

            if j == 0:
                ax3_sub.set_xticks(np.arange(len(model_names)) + 0.25)
                ax3_sub.set_xticklabels(model_names, rotation=45, ha="right")
                ax3_sub.set_ylabel(label)

        ax3.set_title("Model Performance Metrics")
        ax3.grid(True, alpha=0.3)

        # Plot 4: AIC comparison (model selection criterion)
        aic_values = [self.metrics[name]["aic"] for name in self.models.keys()]
        model_names = [self.models[name]["name"] for name in self.models.keys()]

        bars = ax4.bar(range(len(aic_values)), aic_values, color=colors)
        ax4.set_xticks(range(len(model_names)))
        ax4.set_xticklabels(
            [name[:15] for name in model_names], rotation=45, ha="right"
        )
        ax4.set_ylabel("AIC (lower is better)")
        ax4.set_title("Model Selection Criterion (AIC)")
        ax4.grid(True, alpha=0.3)

        # Highlight best model
        best_idx = np.argmin(aic_values)
        bars[best_idx].set_color("gold")
        bars[best_idx].set_edgecolor("black")
        bars[best_idx].set_linewidth(2)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")

        return fig

    def get_best_model(self, criterion="aic"):
        """Get the best model based on specified criterion"""
        if criterion == "aic":
            best_name = min(self.metrics.keys(), key=lambda x: self.metrics[x]["aic"])
        elif criterion == "r2":
            best_name = max(self.metrics.keys(), key=lambda x: self.metrics[x]["r2"])
        elif criterion == "rmse":
            best_name = min(self.metrics.keys(), key=lambda x: self.metrics[x]["rmse"])
        else:
            raise ValueError("Criterion must be 'aic', 'r2', or 'rmse'")

        return best_name, self.models[best_name], self.metrics[best_name]

    def print_detailed_results(self):
        """Print comprehensive results table"""
        print("\n" + "=" * 120)
        print("COMPREHENSIVE MODEL COMPARISON")
        print("=" * 120)

        # Header
        header = f"{'Model':<25} {'R²':<8} {'RMSE':<8} {'MAE':<8} {'AIC':<8} {'Rel.Err(%)':<12} {'Max.Err(%)':<12} {'Params':<8}"
        print(header)
        print("-" * 120)

        # Sort by AIC (best first)
        sorted_models = sorted(self.metrics.items(), key=lambda x: x[1]["aic"])

        for name, metrics in sorted_models:
            model_name = self.models[name]["name"][:24]
            row = f"{model_name:<25} {metrics['r2']:<8.4f} {metrics['rmse']:<8.2f} {metrics['mae']:<8.2f} {metrics['aic']:<8.1f} {metrics['mean_rel_error']:<12.2f} {metrics['max_rel_error']:<12.2f} {metrics['n_params']:<8d}"
            print(row)

        # Best model recommendation
        best_name, best_model, best_metrics = self.get_best_model("aic")
        print("\n" + "=" * 60)
        print(f"RECOMMENDED MODEL: {best_model['name']}")
        print("=" * 60)
        print(f"Selected based on AIC (accounts for model complexity)")
        print(f"AIC: {best_metrics['aic']:.2f}")
        print(f"R²: {best_metrics['r2']:.4f}")
        print(f"RMSE: {best_metrics['rmse']:.2f} cm")
        print(f"Mean Relative Error: {best_metrics['mean_rel_error']:.2f}%")

        return best_name, best_model

    def generate_implementation_code(self, model_name):
        """Generate implementation code for the selected model"""
        model = self.models[model_name]
        coeffs = model["coeffs"]

        if model_name.startswith("poly_"):
            degree = int(model_name.split("_")[1])
            return f"""
# Distance estimation using Polynomial Degree {degree}
DISTANCE_POLY_COEFFS = {coeffs.tolist()}

def estimate_distance_polynomial(height_px):
    if height_px <= 0:
        return float("inf")
    return max(0.0, np.polyval(DISTANCE_POLY_COEFFS, height_px))
"""
        elif model_name == "inverse_power":
            k, p = coeffs
            return f"""
# Distance estimation using Inverse Power Model
DISTANCE_K = {k:.4f}
DISTANCE_P = {p:.4f}

def estimate_distance_inverse_power(height_px):
    if height_px <= 0:
        return float("inf")
    return DISTANCE_K / (height_px ** DISTANCE_P)
"""
        elif model_name == "exponential":
            a, b, c = coeffs
            return f"""
# Distance estimation using Exponential Decay Model
DISTANCE_EXP_A = {a:.6f}
DISTANCE_EXP_B = {b:.6f}
DISTANCE_EXP_C = {c:.6f}

def estimate_distance_exponential(height_px):
    if height_px <= 0:
        return float("inf")
    import numpy as np
    distance = DISTANCE_EXP_A * np.exp(-DISTANCE_EXP_B * height_px) + DISTANCE_EXP_C
    return max(0.0, distance)
"""
        elif model_name == "logarithmic":
            a, b = coeffs
            return f"""
# Distance estimation using Logarithmic Model
DISTANCE_LOG_A = {a:.6f}
DISTANCE_LOG_B = {b:.6f}

def estimate_distance_logarithmic(height_px):
    if height_px <= 0:
        return float("inf")
    import numpy as np
    distance = DISTANCE_LOG_A * np.log(height_px) + DISTANCE_LOG_B
    return max(0.0, distance)
"""
        elif model_name == "original":
            k = coeffs[0]
            return f"""
# Distance estimation using Original Linear Model
DISTANCE_CALIBRATION_CONST = {k:.1f}

def estimate_distance_linear(height_px):
    if height_px <= 0:
        return float("inf")
    return DISTANCE_CALIBRATION_CONST / height_px
"""
        elif model_name == "robust_poly2":
            # Coefficients are [a, b, c] for ax² + bx + c
            a, b, c = coeffs
            return f"""
# Distance estimation using Robust Polynomial (Degree 2)
DISTANCE_ROBUST_A = {a:.6f}  # x² coefficient
DISTANCE_ROBUST_B = {b:.6f}  # x coefficient  
DISTANCE_ROBUST_C = {c:.6f}  # constant term

def estimate_distance_robust_poly2(height_px):
    if height_px <= 0:
        return float("inf")
    distance = DISTANCE_ROBUST_A * height_px**2 + DISTANCE_ROBUST_B * height_px + DISTANCE_ROBUST_C
    return max(0.0, distance)
"""
        else:
            return f"# Implementation for {model['name']} not yet templated"


def main():
    # Your data points
    data_points = [
        (60, 28.2),
        (70, 25.6),
        (80, 24.4),
        (90, 22.2),
        (100, 21.52),
        (110, 20.32),
        (120, 19.9),
        (130, 19),
        (150, 18.1),
    ]

    analyzer = DistanceModelAnalyzer(data_points)

    print("Fitting models...")
    analyzer.fit_all_models()

    print("Calculating metrics...")
    analyzer.calculate_metrics()

    # Create plots
    fig = analyzer.plot_models_comparison()
    output_dir = os.path.dirname(os.path.abspath(__file__))
    fig.savefig(os.path.join(output_dir, "model_comparison.png"), dpi=300, bbox_inches="tight")

    # Print detailed results
    best_name, best_model = analyzer.print_detailed_results()

    # Generate implementation code
    print("\nImplementation code for best model:")
    print(analyzer.generate_implementation_code(best_name))


if __name__ == "__main__":
    main()
