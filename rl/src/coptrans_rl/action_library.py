"""Versioned 80-action A/O/Spin library; units match core.Action exactly."""
from dataclasses import dataclass
import math
import numbers

from .kernel_adapter import Action, stable_hash


@dataclass(frozen=True)
class ActionLibrary:
    actions: tuple
    name: str = "aos_80_v1"

    @classmethod
    def from_contract(cls, contract):
        lengths = [.25*contract.length, .5*contract.length, contract.length]
        actions = []
        for direction in (1, -1):
            for control in (-1., -.5, 0., .5, 1.):
                for length in lengths:
                    actions.append(Action(0, direction*length,
                                          control/contract.limits.r_min_ack))
        for direction in (1, -1):
            for angle in (-90, -60, -30, 0, 30, 60, 90):
                for length in lengths:
                    actions.append(Action(1, direction*length, math.radians(angle)))
        for sign in (1, -1):
            for angle in (15, 30, 60, 90):
                actions.append(Action(2, sign*math.radians(angle), 0.))
        return cls(tuple(actions))

    def __len__(self):
        return len(self.actions)

    def __getitem__(self, index):
        if (isinstance(index, bool) or not isinstance(index, numbers.Integral) or
                not 0 <= index < len(self.actions)):
            raise ValueError("invalid_action_id")
        return self.actions[index]

    @property
    def identity(self):
        return stable_hash({"name": self.name, "actions": [
            [a.mode, a.amount, a.control] for a in self.actions]})
