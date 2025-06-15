import numpy as np
from enum import Enum
from typing import Union, Optional, Dict, Any
import logging
from hook.utils.distance_parameters import (
    DISTANCE_POLY_COEFFS,
    DISTANCE_CALIBRATION_CONST,
    DISTANCE_EXP_A,
    DISTANCE_EXP_B,
    DISTANCE_EXP_C,
    DISTANCE_LOG_A,
    DISTANCE_LOG_B,
    DISTANCE_K,
    DISTANCE_P,
    DISTANCE_ROBUST_A,
    DISTANCE_ROBUST_B,
    DISTANCE_ROBUST_C,
    TARGET_DISTANCE_CM,
    DISTANCE_TOLERANCE_CM,
)


class EstimationMethod(Enum):
    """Enumeration of available distance estimation methods."""

    LINEAR = "linear"
    POLYNOMIAL = "polynomial"
    EXPONENTIAL = "exponential"
    INVERSE_POWER = "inverse_power"
    LOGARITHMIC = "logarithmic"
    ROBUST_POLY2 = "robust_poly2"


class DistanceEstimationError(Exception):
    """Custom exception for distance estimation errors."""

    pass


class DistanceEstimator:

    def __init__(
        self,
        default_method: Union[EstimationMethod, str] = EstimationMethod.EXPONENTIAL,
        validate_inputs: bool = True,
    ):
        """
        Initialize the distance estimator.

        Args:
            default_method: Default estimation method to use
            validate_inputs: Whether to validate input parameters
            log_predictions: Whether to log predictions for monitoring
        """
        self.default_method = self._parse_method(default_method)
        self.validate_inputs = validate_inputs

        self.model_params = self._load_model_parameters()
        self.valid_height_range = (15.0, 35.0)  # Based on your training data
        self.logger = logging.getLogger(__name__)

    def _parse_method(self, method: Union[EstimationMethod, str]) -> EstimationMethod:
        """Parse method input to EstimationMethod enum."""
        if isinstance(method, str):
            try:
                return EstimationMethod(method.lower())
            except ValueError:
                available = [m.value for m in EstimationMethod]
                raise ValueError(
                    f"Invalid method '{method}'. Available methods: {available}"
                )
        elif isinstance(method, EstimationMethod):
            return method
        else:
            raise TypeError(
                f"Method must be string or EstimationMethod enum, got {type(method)}"
            )

    def _load_model_parameters(self) -> Dict[str, Dict[str, Any]]:
        """Load parameters for all models."""
        return {
            EstimationMethod.LINEAR.value: {"k": DISTANCE_CALIBRATION_CONST},
            EstimationMethod.POLYNOMIAL.value: {"coeffs": DISTANCE_POLY_COEFFS},
            EstimationMethod.EXPONENTIAL.value: {
                "a": DISTANCE_EXP_A,
                "b": DISTANCE_EXP_B,
                "c": DISTANCE_EXP_C,
            },
            EstimationMethod.INVERSE_POWER.value: {"k": DISTANCE_K, "p": DISTANCE_P},
            EstimationMethod.LOGARITHMIC.value: {
                "a": DISTANCE_LOG_A,
                "b": DISTANCE_LOG_B,
            },
            EstimationMethod.ROBUST_POLY2.value: {
                "a": DISTANCE_ROBUST_A,
                "b": DISTANCE_ROBUST_B,
                "c": DISTANCE_ROBUST_C,
            },
        }

    def _validate_height(self, height_px: float) -> None:
        """Validate input height parameter."""
        if not isinstance(height_px, (int, float)):
            raise TypeError(f"height_px must be numeric, got {type(height_px)}")

        if height_px <= 0:
            raise ValueError("height_px must be positive")

        if np.isnan(height_px) or np.isinf(height_px):
            raise ValueError("height_px must be finite")

        if not (self.valid_height_range[0] <= height_px <= self.valid_height_range[1]):
            self.logger.warning(
                f"Height {height_px}px is outside training range "
                f"{self.valid_height_range}. Prediction may be unreliable."
            )

    def _estimate_linear(self, height_px: float) -> float:
        """Linear estimation: distance = k / height"""
        k = self.model_params[EstimationMethod.LINEAR.value]["k"]
        return k / height_px

    def _estimate_polynomial(self, height_px: float) -> float:
        """Polynomial estimation using numpy polyval"""
        coeffs = self.model_params[EstimationMethod.POLYNOMIAL.value]["coeffs"]
        return np.polyval(coeffs, height_px)

    def _estimate_exponential(self, height_px: float) -> float:
        """Exponential estimation: distance = a * exp(-b * height) + c"""
        params = self.model_params[EstimationMethod.EXPONENTIAL.value]
        return params["a"] * np.exp(-params["b"] * height_px) + params["c"]

    def _estimate_inverse_power(self, height_px: float) -> float:
        """Inverse power estimation: distance = k / (height^p)"""
        params = self.model_params[EstimationMethod.INVERSE_POWER.value]
        return params["k"] / (height_px ** params["p"])

    def _estimate_logarithmic(self, height_px: float) -> float:
        """Logarithmic estimation: distance = a * log(height) + b"""
        params = self.model_params[EstimationMethod.LOGARITHMIC.value]
        return params["a"] * np.log(height_px) + params["b"]

    def _estimate_robust_poly2(self, height_px: float) -> float:
        """Robust polynomial degree 2: distance = a*height² + b*height + c"""
        params = self.model_params[EstimationMethod.ROBUST_POLY2.value]
        return params["a"] * height_px**2 + params["b"] * height_px + params["c"]

    def estimate_distance(
        self, height_px: float, method: Optional[Union[EstimationMethod, str]] = None
    ) -> float:
        """
        Estimate distance from pixel height using specified method.

        Args:
            height_px: Height of the object in pixels
            method: Estimation method to use (if None, uses default)

        Returns:
            Estimated distance in centimeters

        Raises:
            DistanceEstimationError: If estimation fails
            ValueError: If inputs are invalid
        """
        if method is None:
            method = self.default_method
        else:
            method = self._parse_method(method)

        if self.validate_inputs:
            self._validate_height(height_px)

        try:
            if method == EstimationMethod.LINEAR:
                distance = self._estimate_linear(height_px)
            elif method == EstimationMethod.POLYNOMIAL:
                distance = self._estimate_polynomial(height_px)
            elif method == EstimationMethod.EXPONENTIAL:
                distance = self._estimate_exponential(height_px)
            elif method == EstimationMethod.INVERSE_POWER:
                distance = self._estimate_inverse_power(height_px)
            elif method == EstimationMethod.LOGARITHMIC:
                distance = self._estimate_logarithmic(height_px)
            elif method == EstimationMethod.ROBUST_POLY2:
                distance = self._estimate_robust_poly2(height_px)
            else:
                raise DistanceEstimationError(f"Method {method} not implemented")

            return max(0.0, distance)

        except Exception as e:
            raise DistanceEstimationError(
                f"Distance estimation failed: {str(e)}"
            ) from e

    def set_default_method(self, method: Union[EstimationMethod, str]) -> None:
        """Set the default estimation method."""
        self.default_method = self._parse_method(method)
        self.logger.info(f"Default method set to: {self.default_method.value}")

    def get_available_methods(self) -> list[str]:
        """Get list of available estimation methods."""
        return [method.value for method in EstimationMethod]

    def get_method_info(self, method: Union[EstimationMethod, str]) -> Dict[str, Any]:
        """Get information about a specific method."""
        method_obj = self._parse_method(method)

        method_info = {
            EstimationMethod.LINEAR: {
                "name": "Linear (Inverse)",
                "formula": "distance = k / height",
            },
            EstimationMethod.POLYNOMIAL: {
                "name": "Polynomial",
                "formula": "distance = Σ(aᵢ * heightⁱ)",
            },
            EstimationMethod.EXPONENTIAL: {
                "name": "Exponential Decay",
                "formula": "distance = a * exp(-b * height) + c",
            },
            EstimationMethod.INVERSE_POWER: {
                "name": "Inverse Power",
                "formula": "distance = k / (height^p)",
            },
            EstimationMethod.LOGARITHMIC: {
                "name": "Logarithmic",
                "formula": "distance = a * log(height) + b",
            },
            EstimationMethod.ROBUST_POLY2: {
                "name": "Robust Polynomial (Degree 2)",
                "formula": "distance = a * height² + b * height + c",
            },
        }

        return method_info.get(method_obj, {"name": "Unknown", "formula": "N/A"})


# Example usage
if __name__ == "__main__":
    # Create estimator with exponential method as default
    estimator = DistanceEstimator(
        default_method=EstimationMethod.EXPONENTIAL,
        validate_inputs=True,
    )

    # Test with different methods
    test_height = 25.0

    print(f"Testing with height: {test_height}px")
    print("-" * 50)

    for method in EstimationMethod:
        try:
            distance = estimator.estimate_distance(test_height, method)
            print(f"{method.value:15}: {distance:6.2f} cm")
        except Exception as e:
            print(f"{method.value:15}: Error - {e}")

    # Show method info
    print(f"\nMethod info for {estimator.default_method.value}:")
    info = estimator.get_method_info(estimator.default_method)
    for key, value in info.items():
        print(f"  {key}: {value}")
