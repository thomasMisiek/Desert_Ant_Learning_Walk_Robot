import numpy as np

class CXRing:
    """
    8-neuron allocentric ring model.

    - Sinusoidal activity in [-1, 1]
    - Update weight configurable in [0, 1]
    - Returns decoded allocentric azimuth (deg)
    """

    def __init__(self, update_gain=0.1):
        """
        Parameters
        ----------
        update_gain : float
            Weight of each update in [0, 1].
            0   -> no update
            1   -> full cosine vote
        """
        if not 0.0 <= update_gain <= 2.0:
            raise ValueError("update_gain must be between 0 and 2.")

        self.preferred_directions_deg = np.array(
            [-157.5, -112.5, -67.5, -22.5,
              22.5,   67.5,  112.5, 157.5]
        )

        self.preferred_directions_rad = np.deg2rad(
            self.preferred_directions_deg
        )

        self.activation = np.zeros(8)

        self.max_activation = 1.0
        self.update_gain = update_gain

    # --------------------------------------------------
    # Internal helpers
    # --------------------------------------------------

    def _fit_sinusoid(self):
        """Project onto first Fourier component."""
        cos_component = np.sum(
            self.activation * np.cos(self.preferred_directions_rad)
        )
        sin_component = np.sum(
            self.activation * np.sin(self.preferred_directions_rad)
        )

        amplitude = np.sqrt(
            cos_component**2 + sin_component**2
        ) / 4.0

        phase = np.arctan2(
            sin_component,
            cos_component,
        )

        self.activation = amplitude * np.cos(
            self.preferred_directions_rad - phase
        )

    def _apply_bounds(self):
        """Scale amplitude if it exceeds ±1."""
        max_abs = np.max(np.abs(self.activation))
        if max_abs > self.max_activation:
            self.activation *= self.max_activation / max_abs

    def _decode_population_vector(self):
        """Return continuous allocentric azimuth (deg)."""
        x_comp = np.sum(
            self.activation * np.cos(self.preferred_directions_rad)
        )
        y_comp = np.sum(
            self.activation * np.sin(self.preferred_directions_rad)
        )

        if np.abs(x_comp) < 1e-9 and np.abs(y_comp) < 1e-9:
            return 0.0

        angle_rad = np.arctan2(y_comp, x_comp)
        return np.rad2deg(angle_rad) % 360

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def presentation(self, allocentric_angle_deg):
        """
        Add sinusoidal vote weighted by update_gain.

        Returns
        -------
        azimuth_deg : float
            Decoded allocentric direction (degrees).
        speed : float
            Max activation of the ring (bounded in [0, 1]).
        """
        angle_rad = np.deg2rad(allocentric_angle_deg)

        vote = np.cos(
            self.preferred_directions_rad - angle_rad
        )

        vote *= self.update_gain

        self.activation += vote

        self._fit_sinusoid()
        self._apply_bounds()

        azimuth_deg = self._decode_population_vector()

        speed = np.max(self.activation)

        return azimuth_deg, speed
