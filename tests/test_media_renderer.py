"""Occlusion regressions for the scene image renderer; no game assets required."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location(
    "render_media", Path(__file__).parents[1] / "tools/render_media.py"
)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def square(height, extent):
    return np.array([
        [[-extent, -extent, height], [extent, -extent, height], [extent, extent, height]],
        [[-extent, -extent, height], [extent, extent, height], [-extent, extent, height]],
    ], dtype=np.float64)


class NavigationOcclusionTests(unittest.TestCase):
    def render_road_and_deck(self, navigation_height, elevation=90):
        scene = [
            ("Collision", "bridge deck", square(100, 2)),
            ("Candidate NAVM", "road navigation", square(navigation_height, 1)),
        ]
        return np.array(renderer.render(scene, "Candidate NAVM", None, 160, 160, 0, elevation))

    def navigation_pixels(self, image):
        return np.all(image == renderer.GENERATED, axis=2).sum()

    def test_bridge_deck_hides_road_navigation(self):
        self.assertEqual(self.navigation_pixels(self.render_road_and_deck(0)), 0)

    def test_coplanar_deck_navigation_remains_visible(self):
        self.assertGreater(self.navigation_pixels(self.render_road_and_deck(100)), 100)

    def test_navigation_above_the_deck_is_visible(self):
        self.assertGreater(self.navigation_pixels(self.render_road_and_deck(101)), 100)

    def test_small_burial_is_not_treated_as_coplanar(self):
        self.assertEqual(self.navigation_pixels(self.render_road_and_deck(99.99)), 0)

    def test_oblique_view_uses_collision_depth(self):
        # A broad deck covers the road's projection even from an oblique view.
        scene = [
            ("Collision", "bridge deck", square(1, 4)),
            ("Candidate NAVM", "road navigation", square(0, 1)),
        ]
        image = np.array(renderer.render(scene, "Candidate NAVM", None, 160, 160, 0, 60))
        self.assertEqual(self.navigation_pixels(image), 0)


if __name__ == "__main__":
    unittest.main()
