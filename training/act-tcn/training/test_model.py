"""Tiny ToF-ACT模型层的CPU回归测试。"""

from __future__ import annotations

import unittest

import torch

from training.config import ACTION_CHUNK_FRAMES, LATENT_DIM, MOTOR_COUNT
from training.model import TinyTofAct, count_parameters


class TinyTofActTest(unittest.TestCase):
    def setUp(self) -> None:
        torch.manual_seed(7)
        self.model = TinyTofAct()
        self.tof = torch.randn(2, 12, 18, 8, 8)
        self.state = torch.randn(2, 12, 3)
        self.action = torch.randn(2, 10, 3)
        self.mask = torch.tensor(
            [
                [False] * 10,
                [False] * 6 + [True] * 4,
            ]
        )

    def test_inference_shape_and_parameter_count(self) -> None:
        self.model.eval()
        with torch.no_grad():
            prediction = self.model(self.tof, self.state)

        self.assertEqual(
            tuple(prediction.shape),
            (2, ACTION_CHUNK_FRAMES, MOTOR_COUNT),
        )
        self.assertTrue(torch.isfinite(prediction).all())

        parameter_count = count_parameters(self.model)
        self.assertGreater(parameter_count, 100_000)
        self.assertLess(parameter_count, 350_000)

    def test_training_forward_and_backward(self) -> None:
        self.model.train()
        prediction, mean, logvar = self.model.forward_train(
            self.tof,
            self.state,
            self.action,
            self.mask,
        )
        self.assertEqual(tuple(prediction.shape), (2, 10, 3))
        self.assertEqual(tuple(mean.shape), (2, LATENT_DIM))
        self.assertEqual(tuple(logvar.shape), (2, LATENT_DIM))

        loss = prediction.square().mean() + 0.001 * (
            mean.square().mean() + logvar.square().mean()
        )
        loss.backward()
        gradients = [
            parameter.grad
            for parameter in self.model.parameters()
            if parameter.grad is not None
        ]
        self.assertTrue(gradients)
        self.assertTrue(all(torch.isfinite(gradient).all() for gradient in gradients))

    def test_padded_actions_do_not_change_latent_statistics(self) -> None:
        self.model.eval()
        changed_action = self.action.clone()
        changed_action[1, 6:] = 10000.0

        with torch.no_grad():
            mean_a, logvar_a = self.model.encode_actions(
                self.state,
                self.action,
                self.mask,
            )
            mean_b, logvar_b = self.model.encode_actions(
                self.state,
                changed_action,
                self.mask,
            )

        torch.testing.assert_close(mean_a[1], mean_b[1], rtol=1e-5, atol=1e-6)
        torch.testing.assert_close(logvar_a[1], logvar_b[1], rtol=1e-5, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
