import random
import threading
from copy import deepcopy
from itertools import combinations
from unittest.mock import patch

from django.core.exceptions import ValidationError
from django.db import connection
from django.test import TestCase, TransactionTestCase, skipUnlessDBFeature

from games.game_map import NEIGHBORS, TERRITORIES, validate_game_map
from games.models import Capital, Player, Territory
from games.services import initialize_game

from .utils import create_game, create_user


def get_capital_slugs(game):
    return sorted(Capital.objects.filter(territory__game=game).values_list("territory__slug", flat=True))


class InitializationTests(TestCase):
    def setUp(self):
        self.game = create_game()

    def assert_not_initialized(self, game, status="waiting"):
        game.refresh_from_db()
        self.assertEqual(game.status, status)
        self.assertEqual(game.territories.count(), 0)
        self.assertFalse(Capital.objects.filter(player__game=game).exists())

    def test_successful_initialization(self):
        initialize_game(self.game, rng=random.Random(1))

        self.game.refresh_from_db()
        self.assertEqual(self.game.status, "active")
        self.assertEqual(self.game.territories.count(), len(TERRITORIES))
        validate_game_map(self.game)

        self.assertEqual(Capital.objects.filter(territory__game=self.game).count(), 3)
        for player in self.game.players.all():
            capital = player.get_capital()
            self.assertEqual(capital.health, 3)
            self.assertEqual(capital.territory.owner, player)
            self.assertEqual(list(player.territories.all()), [capital.territory])

        self.assertEqual(self.game.territories.filter(owner__isnull=True).count(), len(TERRITORIES) - 3)
        self.assertFalse(self.game.territories.exclude(score=0).exists())

    def test_capitals_are_not_neighbors(self):
        for seed in range(10):
            with self.subTest(seed=seed):
                game = create_game()
                initialize_game(game, rng=random.Random(seed))

                capitals = [capital.territory for capital in Capital.objects.filter(territory__game=game)]
                for first, second in combinations(capitals, 2):
                    self.assertFalse(first.neighbors.filter(pk=second.pk).exists())

    def test_random_choice_can_be_controlled(self):
        other_game = create_game()

        initialize_game(self.game, rng=random.Random(42))
        initialize_game(other_game, rng=random.Random(42))

        self.assertEqual(get_capital_slugs(self.game), get_capital_slugs(other_game))

    def test_wrong_number_of_players(self):
        for count in [0, 2]:
            with self.subTest(players=count):
                game = create_game(players=count)

                with self.assertRaises(ValidationError):
                    initialize_game(game)

                self.assert_not_initialized(game)

    def test_wrong_status(self):
        for status in ["active", "completed"]:
            with self.subTest(status=status):
                game = create_game(status=status)

                with self.assertRaises(ValidationError):
                    initialize_game(game)

                self.assert_not_initialized(game, status=status)

    def test_invalid_map_definition(self):
        neighbors = deepcopy(NEIGHBORS)
        neighbors["skali"].append("skali")

        with patch("games.game_map.NEIGHBORS", neighbors):
            with self.assertRaises(ValidationError):
                initialize_game(self.game)

        self.assert_not_initialized(self.game)

    def test_no_possible_capitals(self):
        # every territory is a neighbor of every other territory
        territories = {f"t{number}": f"Territory {number}" for number in range(9)}
        neighbors = {slug: [other for other in territories if other != slug] for slug in territories}

        with patch("games.game_map.TERRITORIES", territories), patch("games.game_map.NEIGHBORS", neighbors):
            with self.assertRaises(ValidationError):
                initialize_game(self.game)

        self.assert_not_initialized(self.game)

    def test_second_initialization_is_refused(self):
        initialize_game(self.game)
        territories = list(self.game.territories.values_list("id", "owner_id"))
        capitals = list(Capital.objects.filter(territory__game=self.game).values_list("id", "player_id", "territory_id"))

        with self.assertRaises(ValidationError):
            initialize_game(self.game)

        self.assertEqual(list(self.game.territories.values_list("id", "owner_id")), territories)
        self.assertEqual(
            list(Capital.objects.filter(territory__game=self.game).values_list("id", "player_id", "territory_id")),
            capitals,
        )

    def test_partial_state_is_not_repaired(self):
        Territory.objects.create(game=self.game, name="Скали", slug="skali")

        with self.assertRaises(ValidationError):
            initialize_game(self.game)

        self.game.refresh_from_db()
        self.assertEqual(self.game.status, "waiting")
        self.assertEqual(list(self.game.territories.values_list("slug", flat=True)), ["skali"])
        self.assertFalse(Capital.objects.exists())

    def test_error_during_initialization_is_rolled_back(self):
        with patch.object(Capital, "save", side_effect=RuntimeError("database error")):
            with self.assertRaises(RuntimeError):
                initialize_game(self.game)

        self.assert_not_initialized(self.game)

    def test_games_are_independent(self):
        other_game = create_game()
        initialize_game(self.game)
        initialize_game(other_game)

        self.assertEqual(
            set(self.game.territories.values_list("slug", flat=True)),
            set(other_game.territories.values_list("slug", flat=True)),
        )
        self.assertFalse(
            set(self.game.territories.values_list("id", flat=True))
            & set(other_game.territories.values_list("id", flat=True))
        )

        territory = self.game.territories.get(slug="madara")
        territory.score = 300
        territory.save()
        self.assertEqual(other_game.territories.get(slug="madara").score, 0)

    def test_no_side_changes(self):
        territories_before = deepcopy(TERRITORIES)
        neighbors_before = deepcopy(NEIGHBORS)
        Player.objects.filter(game=self.game).update(score=7)

        initialize_game(self.game)

        self.assertEqual(TERRITORIES, territories_before)
        self.assertEqual(NEIGHBORS, neighbors_before)
        self.assertEqual(list(self.game.players.values_list("score", flat=True)), [7, 7, 7])
        self.assertEqual(self.game.rounds.count(), 0)

    def test_players_can_change_only_before_start(self):
        player = self.game.players.first()
        self.game.remove_player(player)
        self.game.add_player(create_user("new_player"), player.color)

        initialize_game(self.game)

        with self.assertRaises(ValidationError):
            self.game.add_player(create_user("late_player"), "red")
        with self.assertRaises(ValidationError):
            self.game.remove_player(self.game.players.first())
        self.assertEqual(self.game.players.count(), 3)


@skipUnlessDBFeature("has_select_for_update")
class ConcurrentInitializationTests(TransactionTestCase):
    # SQLite does not support select_for_update, so this test runs only on a database like PostgreSQL
    def test_two_starts_at_the_same_time(self):
        game = create_game()
        errors = []

        def start():
            try:
                initialize_game(game)
            except ValidationError as error:
                errors.append(error)
            finally:
                connection.close()

        threads = [threading.Thread(target=start) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        game.refresh_from_db()
        self.assertEqual(game.status, "active")
        self.assertEqual(len(errors), 1)
        self.assertEqual(game.territories.count(), len(TERRITORIES))
        self.assertEqual(Capital.objects.filter(territory__game=game).count(), 3)
