"""Small two-map CNN and explicit state/history encoder for MaskablePPO."""
import torch
from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class PlanningFeatures(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=128):
        super().__init__(observation_space, features_dim)

        def image_encoder(channels):
            return nn.Sequential(
                nn.Conv2d(channels, 16, 5, 2, 2), nn.ReLU(),
                nn.Conv2d(16, 32, 3, 2, 1), nn.ReLU(),
                nn.Conv2d(32, 32, 3, 2, 1), nn.ReLU(),
                nn.AdaptiveAvgPool2d((2, 2)), nn.Flatten(),
                nn.Linear(128, 64), nn.ReLU())
        self.local = image_encoder(4)
        self.global_map = image_encoder(5)
        self.state = nn.Sequential(nn.Linear(60, 64), nn.ReLU())
        self.mask = nn.Sequential(nn.Linear(80, 32), nn.ReLU())
        self.combine = nn.Sequential(nn.Linear(224, features_dim), nn.ReLU())

    def forward(self, observations):
        return self.combine(torch.cat([
            self.local(observations["local_map"]),
            self.global_map(observations["global_map"]),
            self.state(observations["state"]),
            self.mask(observations["valid_action_mask"])], dim=1))
