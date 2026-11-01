import numpy as np
from scipy.spatial.distance import cosine
import numpy as np
import scipy.sparse as sp
from matplotlib import pyplot as plt
from scipy.signal import argrelextrema
from typing import Tuple, List, Optional
import numpy as np
import cv2
import matplotlib.pyplot as plt
from typing import Tuple









def _closest_angle_index(angles, angle):
    diffs = np.abs((angles - angle + 180) % 360 - 180)
    return int(np.argmin(diffs))






class GreenVisualPipeline:
    def __init__(self, resolution, crop_bounds):
        self.resolution = resolution
        self.crop_bounds = crop_bounds

    def _center_surround(
        self,
        feature: np.ndarray,
        sigma_center: float = 1.0,
        sigma_surround: float = 4.0,
    ) -> np.ndarray:

        feature = np.asarray(feature, dtype=np.float32)

        if feature.ndim != 2:
            raise ValueError("Feature map must be 2D, got shape: ", feature.shape)

        center = cv2.GaussianBlur(feature, (0, 0), sigmaX=sigma_center)
        surround = cv2.GaussianBlur(feature, (0, 0), sigmaX=sigma_surround)

        return np.abs(center - surround)

    def presentation(self, img, learning=False):

        # plt.imshow(img)
        # plt.show()
        # === Resize upfront to 360H and 100V===
        img_resized = cv2.resize(img, (360, 100), interpolation=cv2.INTER_AREA)

        # === Convert to float32 once ===
        img_resized = img_resized.astype(np.float32)
        # plt.imshow(img_resized/255)
        # plt.show()


        # === Extract green channel and crop ===
        green = img_resized[:, :, 1]

        cropped = green[:, self.crop_bounds[0]:self.crop_bounds[1]]

        img_resized = cv2.resize(cropped, (36, 36), interpolation=cv2.INTER_AREA)

        # === Mean subtraction ===
        #normalized = green - np.mean(green, dtype=np.float32)
        #normalized += np.random.normal(0,0.3,normalized.shape)
        # plt.imshow(normalized)
        # plt.show()
        #
        # === Lateral inhibition ===
        inhibited = self._center_surround(img_resized)#lat_inh(normalized)
        # plt.imshow(inhibited)
        # plt.show()

        # === Local contrast normalization ===
        #lcn_output = local_contrast_normalization(inhibited)
        #lcn_output = normalized
        # plt.imshow(lcn_output)
        # plt.show()



        # === Resize and flatten ===
        output_resized = cv2.resize(inhibited  , self.resolution, interpolation=cv2.INTER_AREA)
        return output_resized.flatten()








class MB_8_Nest:
    def __init__(self, n_kcs, n_input, connections_per_kc, lr, sparsity=0.05):
        self.n_kcs = n_kcs
        self.n_input = n_input
        self.connections_per_kc = connections_per_kc
        self.lr = lr
        self.sparsity = sparsity

        # 8 MBONs and their preferred directions
        self.mbon_angles = np.array([-157.5, -112.5, -67.5, -22.5, 22.5, 67.5, 112.5, 157.5]
)
        self.n_mbons = len(self.mbon_angles)

        # Weight matrix: each row is one MBON's weights to KCs
        self.weights = np.random.rand(self.n_mbons, self.n_kcs)#np.ones((self.n_mbons, self.n_kcs))
        self.weight_buffers = np.zeros_like(self.weights)  # Buffer for each MBON

        # KC activation tracking
        self.acts = np.zeros(self.n_kcs)

        # MBON usage tracking
        self.mbon_learning_counts = np.zeros(self.n_mbons, dtype=int)
        self.mbon_retrieval_counts = np.zeros(self.n_mbons, dtype=int)

        # Build sparse binary connectivity matrix (KC ⇐ input)
        rows, cols, data = [], [], []
        for i in range(self.n_kcs):
            selected_inputs = np.random.choice(n_input, connections_per_kc, replace=False)
            rows.extend([i] * connections_per_kc)
            cols.extend(selected_inputs)
            data.extend([1] * connections_per_kc)
        self.kcs = sp.csr_matrix((data, (rows, cols)), shape=(self.n_kcs, self.n_input))

        # Output
        self.activation = 0
        self.selected_angle = None

    def _get_closest_mbon_index(self, angle):
        """Return index of MBON whose preferred angle is closest to the input angle (degrees)."""
        diffs = np.abs((self.mbon_angles - angle + 180) % 360 - 180)
        return np.argmin(diffs)

    def presentation(self, input_vector, learning=False,
                     angle_to_truth=None, angle_to_north=None, timestep=None):

        # KC activation
        kc_activity = self.kcs.dot(input_vector)
        num_active = int(self.sparsity * self.n_kcs)

        top_indices = np.argpartition(kc_activity, -num_active)[-num_active:]
        output_vector = np.zeros_like(kc_activity)
        output_vector[top_indices] = 1

        self.acts += output_vector

        if learning and angle_to_truth is not None:
            #print("HAHAHA")
            idx = self._get_closest_mbon_index(angle_to_truth)
            self.mbon_learning_counts[idx] += 1

            # Buffer the learning update for that MBON (Hebbian depression)
            self.weight_buffers[idx] -= self.lr * output_vector

            self.selected_angle = self.mbon_angles[idx]
            #print(self.selected_angle)
            return [self.selected_angle]

        else:
            # Apply current weights (excluding buffer)
            activations = self.weights @ output_vector

            # ---- Population vector computation (for visualization only) ----
            # 8 vectors, preferred directions = self.mbon_angles
            # ---- Population vector computation (for visualization only) ----
            magnitudes = activations / 1000.0

            # convert preferred directions to radians (circular space)
            angles_rad = np.deg2rad(self.mbon_angles)

            vx = np.sum(magnitudes * np.cos(angles_rad))
            vy = np.sum(magnitudes * np.sin(angles_rad))

            resulting_amplitude = np.hypot(vx, vy)
            resulting_direction = np.rad2deg(np.arctan2(vy, vx)) % 360.0

            # print(
            #     f"Population vector → amplitude: {resulting_amplitude:.4f}, "
            #     f"direction: {resulting_direction:.2f}°"
            # )
            # ---------------------------------------------------------------

            # ---------------------------------------------------------------

            min_value = np.min(activations)
            tied_indices = np.where(activations == min_value)[0]
            min_idx = np.random.choice(tied_indices)  # Randomly break ties

            self.mbon_retrieval_counts[min_idx] += 1
            self.activation = activations[min_idx]
            self.selected_angle = self.mbon_angles[min_idx]
            # print()
            # print(activations)
            # print(self.selected_angle)

            return [self.selected_angle]


    def consolidate(self):
        """Apply all buffered weight updates to main weights and clear the buffer."""
        self.weights += self.weight_buffers
        self.weights = np.clip(self.weights, 0, 2)
        self.weight_buffers[:] = 0  # Reset all buffers

    def nest_return(self):
        """No-op placeholder for interface compatibility."""
        self.consolidate()

    def clear_memory(self):
        """Clear activation counts and weight buffers."""
        self.acts[:] = 0
        self.weight_buffers[:] = 0
        self.activation = 0
        self.selected_angle = None







class MB_8_Nest_PM:
    """
    Exact semantic replacement of MB_8_Nest using PM memories.

    Rules:
    - Learning -> short-term memory only
    - Consolidation ONLY in nest_return()
    - Retrieval:
        * If NO long-term memory exists -> random angle
        * If multiple PMs tie -> random among ties
    """

    def __init__(self):
        self.mbon_angles = np.array(
            [-157.5, -112.5, -67.5, -22.5, 22.5, 67.5, 112.5, 157.5]
        )

        # Long-term and short-term PM memories
        self.memories = [[] for _ in range(8)]
        self.short_term_memories = [[] for _ in range(8)]

        self.selected_angle = None
        self.activation = 0

    def presentation(
        self,
        input_vector,
        learning=False,
        angle_to_truth=None,
        angle_to_north=None,
        timestep=None,
    ):
        input_vector = np.asarray(input_vector)

        # ---------------- LEARNING ----------------
        if learning and angle_to_truth is not None:
            idx = _closest_angle_index(self.mbon_angles, angle_to_truth)
            self.short_term_memories[idx].append(input_vector.copy())
            self.selected_angle = self.mbon_angles[idx]
            return self.selected_angle

        # ---------------- RETRIEVAL ----------------

        # 1️⃣ Check if ANY long-term memory exists
        if not any(len(mem) > 0 for mem in self.memories):
            # No LTM anywhere → random direction
            idx = np.random.randint(0, 8)
            self.selected_angle = self.mbon_angles[idx]
            self.activation = np.inf
            return self.selected_angle

        # 2️⃣ Compute PM errors (LTM + STM)
        pm_errors = np.full(8, np.inf)

        for i in range(8):
            combined = self.memories[i] #+ self.short_term_memories[i]
            if not combined:
                continue

            errors = [
                np.abs(mem - input_vector).sum()
                for mem in combined
            ]
            pm_errors[i] = min(errors)

        # 3️⃣ Select randomly among lowest-error PMs
        min_error = np.min(pm_errors)
        tied_indices = np.where(pm_errors == min_error)[0]
        chosen_idx = np.random.choice(tied_indices)

        self.activation = min_error
        self.selected_angle = self.mbon_angles[chosen_idx]
        return [self.selected_angle]

    def consolidate(self):
        """Alias for nest_return."""
        self.nest_return()

    def nest_return(self):
        """Explicit Nest consolidation."""
        for i in range(8):
            self.memories[i].extend(self.short_term_memories[i])
            self.short_term_memories[i].clear()

    def clear_memory(self):
        self.memories = [[] for _ in range(8)]
        self.short_term_memories = [[] for _ in range(8)]
        self.selected_angle = None
        self.activation = 0
