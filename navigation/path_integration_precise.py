import numpy as np
# Constants and CX model
N_COLUMNS = 8
ANGLES = np.linspace(0 + np.pi / 4, 2 * np.pi + np.pi / 4, N_COLUMNS, endpoint=False)
ROT45 = np.pi / 4  # 45 degrees in radians


class CX:
    def __init__(self, directional_noise_std, stride_noise_std):
        self.cpu4_mem_gain = 0.05
        self.cpu4_mem = np.zeros(N_COLUMNS)
        self.directional_noise_std = directional_noise_std
        self.stride_noise_std = stride_noise_std


    def tb1_output(self, theta):
        return (1.0 + np.cos(np.pi + ANGLES + theta)) - 0.995

    def cpu4_update(self, cpu4_mem, tb1, tn1):
        motion_vector = tn1 * tb1
        cpu4_mem += self.cpu4_mem_gain * motion_vector
        return cpu4_mem

    def update_and_decode(self, heading_rad, flow):
        noisy_heading = heading_rad #+ np.deg2rad(np.random.normal(0, self.directional_noise_std))
        flow_noisy = flow * (1.0)# + np.random.normal(0, self.stride_noise_std))
        tb1 = self.tb1_output(noisy_heading)
        self.cpu4_mem = self.cpu4_update(self.cpu4_mem, tb1, flow_noisy)

        cpu4_ring = self.cpu4_mem.copy()
        if cpu4_ring.sum() > 0:
            x_comp = np.sum(cpu4_ring * np.cos(ANGLES)) / cpu4_ring.sum()
            y_comp = -np.sum(cpu4_ring * np.sin(ANGLES)) / cpu4_ring.sum()
            return np.rad2deg(np.arctan2(y_comp, x_comp)) % 360
        else:
            return np.rad2deg(heading_rad) % 360  # fallback


    def update_and_decode_egocentric(self, heading_rad, flow):
        """
        Returns:
            allocentric_deg: direction of nest in world frame
            egocentric_deg: direction of nest relative to agent heading
        """
        allocentric_deg = self.update_and_decode(heading_rad, flow)

        # Convert to radians
        allocentric_rad = np.deg2rad(allocentric_deg)

        # Egocentric = allocentric - heading
        egocentric_rad = allocentric_rad - heading_rad

        # Wrap to [-pi, pi]
        egocentric_rad = (egocentric_rad + np.pi) % (2 * np.pi) - np.pi

        return np.rad2deg(egocentric_rad)

    def get_home_distance(self):
        """
        Compute distance from the CPU4 ring using vector amplitude.

        Returns:
            distance (float): magnitude of the population vector
        """
        cpu4_ring = self.cpu4_mem.copy()

        if cpu4_ring.sum() > 0:
            # Compute population vector
            x_comp = np.sum(cpu4_ring * np.cos(ANGLES))
            y_comp = -np.sum(cpu4_ring * np.sin(ANGLES))

            # Magnitude = distance signal
            distance = np.sqrt(x_comp**2 + y_comp**2)

            return distance/10
        else:
            return 0.0
