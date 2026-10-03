from django.core.exceptions import ValidationError
from django.db import IntegrityError, transaction
from django.test import TestCase

from games.models import Game, Player

from .utils import create_game, create_user


class PlayerTests(TestCase):
    def setUp(self):
        self.game = Game.objects.create()

    def test_add_three_players(self):
        self.game.add_player(create_user("ivan"), "red")
        self.game.add_player(create_user("maria"), "green")
        self.game.add_player(create_user("petar"), "blue")

        self.assertEqual(self.game.players.count(), 3)
        self.assertEqual(
            set(self.game.players.values_list("color", flat=True)),
            {"red", "green", "blue"},
        )
        self.assertTrue(all(player.score == 0 for player in self.game.players.all()))

    def test_invalid_player(self):
        user = create_user("ivan")
        Player.objects.create(game=self.game, user=user, color="red")
        cases = {
            "same user": {"user": user, "color": "green"},
            "same color": {"user": create_user("maria"), "color": "red"},
            "negative score": {"user": create_user("petar"), "color": "blue", "score": -1},
        }

        for name, data in cases.items():
            with self.subTest(name):
                with self.assertRaises(IntegrityError):
                    with transaction.atomic():
                        Player.objects.create(game=self.game, **data)

        self.assertEqual(self.game.players.count(), 1)

    def test_same_user_and_color_in_different_games(self):
        user = create_user("ivan")
        other_game = Game.objects.create()

        self.game.add_player(user, "red")
        other_game.add_player(user, "red")

        self.assertEqual(Player.objects.filter(user=user).count(), 2)

    def test_cannot_add_fourth_player(self):
        game = create_game(players=3)

        with self.assertRaises(ValidationError):
            game.add_player(create_user("fourth"), "red")

        self.assertEqual(game.players.count(), 3)

    def test_cannot_join_active_game(self):
        game = create_game(players=2, status="active")

        with self.assertRaises(ValidationError):
            game.add_player(create_user("late_player"), "blue")
