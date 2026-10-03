from django.core.exceptions import ValidationError
from django.db import IntegrityError, transaction
from django.test import TestCase

from games.models import Round

from .utils import create_game


class RoundTests(TestCase):
    def setUp(self):
        self.game = create_game(status="active")
        self.player = self.game.players.first()

    def test_create_round(self):
        game_round = self.game.create_round("city_capture")

        self.assertEqual(game_round.game, self.game)
        self.assertEqual(game_round.number, 1)
        self.assertEqual(game_round.type, "city_capture")
        self.assertEqual(game_round.status, "pending")
        self.assertIsNone(game_round.winner)
        self.assertIsNone(game_round.completed_at)

    def test_cannot_create_round_in_waiting_game(self):
        game = create_game(status="waiting")

        with self.assertRaises(ValidationError):
            game.create_round("battle")

    def test_complete_round(self):
        game_round = self.game.create_round("battle")
        game_round.start()

        game_round.complete(self.player)

        game_round.refresh_from_db()
        self.assertEqual(game_round.status, "completed")
        self.assertEqual(game_round.winner, self.player)
        self.assertIsNotNone(game_round.completed_at)

    def test_winner_must_be_from_same_game(self):
        other_player = create_game(status="active").players.first()
        game_round = self.game.create_round("battle")
        game_round.start()

        with self.assertRaises(ValidationError):
            game_round.complete(other_player)

        game_round.refresh_from_db()
        self.assertEqual(game_round.status, "active")
        self.assertIsNone(game_round.winner)

    def test_get_current_round(self):
        first_round = self.game.create_round("city_capture")
        first_round.start()
        first_round.complete(self.player)
        second_round = self.game.create_round("battle")

        self.assertEqual(self.game.get_current_round(), second_round)

    def test_new_round_only_after_current_is_completed(self):
        self.game.create_round("city_capture")

        with self.assertRaises(ValidationError):
            self.game.create_round("battle")

    def test_round_number_is_unique_in_game(self):
        Round.objects.create(game=self.game, number=1, type="battle")

        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Round.objects.create(game=self.game, number=1, type="bonus")

        other_game = create_game(status="active")
        Round.objects.create(game=other_game, number=1, type="bonus")

    def test_only_one_active_round_per_game(self):
        Round.objects.create(game=self.game, number=1, type="battle", status="active")

        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Round.objects.create(game=self.game, number=2, type="bonus", status="active")
