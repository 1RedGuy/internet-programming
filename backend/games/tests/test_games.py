from django.core.exceptions import ValidationError
from django.test import TestCase

from games.models import Game
from games.services import initialize_game

from .utils import create_game, create_user


class GameTests(TestCase):
    def test_create_game(self):
        game = Game.objects.create()

        self.assertEqual(game.status, "waiting")
        self.assertIsNotNone(game.created_at)
        self.assertFalse(game.is_active())
        self.assertFalse(game.is_completed())

    def test_start_game_with_three_players(self):
        game = create_game(players=3)

        initialize_game(game)

        game.refresh_from_db()
        self.assertEqual(game.status, "active")
        self.assertTrue(game.is_active())

    def test_cannot_start_without_three_players(self):
        game = create_game(players=2)

        with self.assertRaises(ValidationError):
            initialize_game(game)

        game.refresh_from_db()
        self.assertEqual(game.status, "waiting")

    def test_cannot_start_active_game_again(self):
        game = create_game(status="active")

        with self.assertRaises(ValidationError):
            initialize_game(game)

    def test_complete_game(self):
        game = create_game(status="active")

        game.complete()

        game.refresh_from_db()
        self.assertTrue(game.is_completed())

    def test_completed_game_has_no_new_rounds_or_players(self):
        game = create_game(players=2, status="completed")

        with self.assertRaises(ValidationError):
            game.create_round("battle")
        with self.assertRaises(ValidationError):
            game.add_player(create_user("late_player"), "blue")

        self.assertEqual(game.rounds.count(), 0)
        self.assertEqual(game.players.count(), 2)
