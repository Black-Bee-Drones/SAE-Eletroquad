import numpy as np
from enum import Enum
from typing import Union, Optional, Dict, Any
import logging
from slalom.utils.distance_parameters import (
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


class BeamDistanceEstimator:

    def __init__(
        self,
        default_method: Union[EstimationMethod, str] = EstimationMethod.LINEAR,
        validate_inputs: bool = False,
    ):
        """
        Initialize the beam distance estimator.

        Args:
            default_method: Default estimation method to use
            validate_inputs: Whether to validate input parameters
        """
        self.default_method = self._parse_method(default_method)
        self.validate_inputs = validate_inputs

        self.model_params = self._load_model_parameters()
        self.valid_width_range = (20.0, 300.0)  # Expected beam width range in pixels
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

    def _validate_width(self, width_px: float) -> None:
        """Validate input width parameter."""
        if not isinstance(width_px, (int, float)):
            raise TypeError(f"width_px must be numeric, got {type(width_px)}")

        if width_px <= 0:
            raise ValueError("width_px must be positive")

        if np.isnan(width_px) or np.isinf(width_px):
            raise ValueError("width_px must be finite")

        if not (self.valid_width_range[0] <= width_px <= self.valid_width_range[1]):
            self.logger.warning(
                f"Width {width_px}px is outside expected range "
                f"{self.valid_width_range}. Prediction may be unreliable."
            )

    def _estimate_linear(self, width_px: float) -> float:
        """Linear estimation: distance = k / width"""
        k = self.model_params[EstimationMethod.LINEAR.value]["k"]
        return k / width_px

    def _estimate_polynomial(self, width_px: float) -> float:
        """Polynomial estimation using numpy polyval"""
        coeffs = self.model_params[EstimationMethod.POLYNOMIAL.value]["coeffs"]
        return np.polyval(coeffs, width_px)

    def _estimate_exponential(self, width_px: float) -> float:
        """Exponential estimation: distance = a * exp(-b * width) + c"""
        params = self.model_params[EstimationMethod.EXPONENTIAL.value]
        return params["a"] * np.exp(-params["b"] * width_px) + params["c"]

    def _estimate_inverse_power(self, width_px: float) -> float:
        """Inverse power estimation: distance = k / (width^p)"""
        params = self.model_params[EstimationMethod.INVERSE_POWER.value]
        return params["k"] / (width_px ** params["p"])

    def _estimate_logarithmic(self, width_px: float) -> float:
        """Logarithmic estimation: distance = a * log(width) + b"""
        params = self.model_params[EstimationMethod.LOGARITHMIC.value]
        return params["a"] * np.log(width_px) + params["b"]

    def _estimate_robust_poly2(self, width_px: float) -> float:
        """Robust polynomial degree 2: distance = a*width² + b*width + c"""
        params = self.model_params[EstimationMethod.ROBUST_POLY2.value]
        return params["a"] * width_px**2 + params["b"] * width_px + params["c"]

    def estimate_distance(
        self, width_px: float, method: Optional[Union[EstimationMethod, str]] = None
    ) -> float:
        """
        Estimate distance from pixel width using specified method.

        Args:
            width_px: Width of the beam in pixels
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
            self._validate_width(width_px)

        try:
            if method == EstimationMethod.LINEAR:
                distance = self._estimate_linear(width_px)
            elif method == EstimationMethod.POLYNOMIAL:
                distance = self._estimate_polynomial(width_px)
            elif method == EstimationMethod.EXPONENTIAL:
                distance = self._estimate_exponential(width_px)
            elif method == EstimationMethod.INVERSE_POWER:
                distance = self._estimate_inverse_power(width_px)
            elif method == EstimationMethod.LOGARITHMIC:
                distance = self._estimate_logarithmic(width_px)
            elif method == EstimationMethod.ROBUST_POLY2:
                distance = self._estimate_robust_poly2(width_px)
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

    def is_at_target_distance(self, width_px: float, target_distance_cm: float = None) -> bool:
        """
        Check if the beam is at the target distance.

        Args:
            width_px: Current beam width in pixels
            target_distance_cm: Target distance in cm (uses default if None)

        Returns:
            True if at target distance within tolerance
        """
        if target_distance_cm is None:
            target_distance_cm = TARGET_DISTANCE_CM

        current_distance = self.estimate_distance(width_px)
        return abs(current_distance - target_distance_cm) <= DISTANCE_TOLERANCE_CM


# Example usage
if __name__ == "__main__":
    # Create estimator with linear method as default
    estimator = BeamDistanceEstimator(
        default_method=EstimationMethod.LINEAR,
        validate_inputs=True,
    )

    # Test with different beam widths
    test_widths = [50, 100, 150, 200]

    print("Testing beam distance estimation:")
    print("-" * 50)

    for width in test_widths:
        try:
            distance = estimator.estimate_distance(width)
            at_target = estimator.is_at_target_distance(width)
            print(f"Width: {width:3d}px -> Distance: {distance:6.1f}cm (Target: {at_target})")
        except Exception as e:
            print(f"Width: {width:3d}px -> Error: {e}")

    print(f"\nTarget distance: {TARGET_DISTANCE_CM}cm ± {DISTANCE_TOLERANCE_CM}cm") 