"""Spatio-Temporal Attention Module (STAM) feature extractor for SAC.

Reshapes the flat stacked-frame observation into a sequence of timesteps and
applies a single Transformer encoder layer so the policy can attend across the
observation history before feeding into actor/critic MLPs.
"""

import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# Must stay in sync with RobotNavEnv history length. The per-step feature size
# is inferred from the observation space so from-scratch runs can enable the
# enhanced observation without rewriting the policy extractor.
_HISTORY_LENGTH = 8
_SINGLE_OBS_SIZE = 40  # legacy default: 36 lidar + goal_dist + goal_angle + velocities


class STAMFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Box, features_dim: int = 64) -> None:
        super().__init__(observation_space, features_dim)

        self.history_length = _HISTORY_LENGTH
        obs_dim = int(observation_space.shape[0])
        if obs_dim % self.history_length != 0:
            raise ValueError(
                "STAM observation dimension must be divisible by history length "
                f"{self.history_length}, got {obs_dim}"
            )
        self.single_obs_size = obs_dim // self.history_length
        embed_dim = features_dim

        self.input_proj = nn.Linear(self.single_obs_size, embed_dim)

        # Learnable positional encoding: one vector per timestep
        self.pos_encoding = nn.Parameter(torch.zeros(1, _HISTORY_LENGTH, embed_dim))
        nn.init.trunc_normal_(self.pos_encoding, std=0.02)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=4,
            dim_feedforward=128,
            dropout=0.1,
            batch_first=True,
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=1)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        batch = observations.shape[0]
        # (batch, history*single) -> (batch, history, single)
        x = observations.view(batch, self.history_length, self.single_obs_size)
        x = self.input_proj(x) + self.pos_encoding
        x = self.transformer(x)
        # Use the last timestep as the summary — most recent observation matters most for control
        return x[:, -1, :]
