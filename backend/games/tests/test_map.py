from copy import deepcopy

from django.core.exceptions import ValidationError
from django.test import SimpleTestCase

from games.game_map import NEIGHBORS, TERRITORIES, get_capital_options, validate_map


def make_ring(size):
    territories = {f"t{number}": f"Territory {number}" for number in range(size)}
    neighbors = {
        f"t{number}": [f"t{(number - 1) % size}", f"t{(number + 1) % size}"]
        for number in range(size)
    }
    return territories, neighbors


class MapDefinitionTests(SimpleTestCase):
    def test_project_map_is_valid(self):
        validate_map(TERRITORIES, NEIGHBORS)

        self.assertEqual(len(TERRITORIES), 12)

    def test_map_size(self):
        for size in [9, 21]:
            with self.subTest(size=size):
                validate_map(*make_ring(size))

        for size in [6, 10, 24]:
            with self.subTest(size=size):
                with self.assertRaises(ValidationError):
                    validate_map(*make_ring(size))

    def test_invalid_neighbors(self):
        def self_reference(neighbors):
            neighbors["skali"].append("skali")

        def not_symmetric(neighbors):
            neighbors["skali"].append("leventa")

        def duplicate(neighbors):
            neighbors["skali"].append("dunaviya")

        def unknown_territory(neighbors):
            neighbors["skali"].append("nowhere")

        def one_neighbor(neighbors):
            neighbors["skali"].remove("sredets")
            neighbors["sredets"].remove("skali")

        for change in [self_reference, not_symmetric, duplicate, unknown_territory, one_neighbor]:
            with self.subTest(change.__name__):
                neighbors = deepcopy(NEIGHBORS)
                change(neighbors)

                with self.assertRaises(ValidationError):
                    validate_map(TERRITORIES, neighbors)

    def test_duplicate_names(self):
        territories = dict(TERRITORIES)
        territories["pirina"] = "Скали"

        with self.assertRaises(ValidationError):
            validate_map(territories, NEIGHBORS)

    def test_map_must_be_connected(self):
        # three separate triangles
        territories = {f"t{number}": f"Territory {number}" for number in range(9)}
        neighbors = {}
        for start in [0, 3, 6]:
            group = [f"t{start}", f"t{start + 1}", f"t{start + 2}"]
            for slug in group:
                neighbors[slug] = [other for other in group if other != slug]

        with self.assertRaises(ValidationError):
            validate_map(territories, neighbors)

    def test_map_needs_three_capitals_that_are_not_neighbors(self):
        # every territory is a neighbor of every other territory
        territories = {f"t{number}": f"Territory {number}" for number in range(9)}
        neighbors = {slug: [other for other in territories if other != slug] for slug in territories}

        self.assertEqual(get_capital_options(neighbors), [])
        with self.assertRaises(ValidationError):
            validate_map(territories, neighbors)

    def test_capital_options_are_not_neighbors(self):
        for first, second, third in get_capital_options(NEIGHBORS):
            self.assertNotIn(second, NEIGHBORS[first])
            self.assertNotIn(third, NEIGHBORS[first])
            self.assertNotIn(third, NEIGHBORS[second])
