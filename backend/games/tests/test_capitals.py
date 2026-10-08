from django.core.exceptions import ValidationError
from django.db import IntegrityError, transaction
from django.test import TestCase

from games.models import Capital, Territory

from .utils import create_game


class CapitalTests(TestCase):
    def setUp(self):
        self.game = create_game()
        self.player = self.game.players.first()
        self.territory = Territory.objects.create(
            game=self.game,
            name="Скали",
            slug="skali",
            owner=self.player,
        )

    def test_create_capital(self):
        capital = Capital(territory=self.territory, player=self.player)

        capital.full_clean()
        capital.save()

        self.assertEqual(capital.health, 3)
        self.assertEqual(capital.game, self.game)

    def test_one_capital_per_player_and_territory(self):
        Capital.objects.create(territory=self.territory, player=self.player)
        other_territory = Territory.objects.create(game=self.game, name="Езера", slug="ezera", owner=self.player)
        other_player = self.game.players.exclude(pk=self.player.pk).first()

        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Capital.objects.create(territory=other_territory, player=self.player)
        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Capital.objects.create(territory=self.territory, player=other_player)

    def test_player_from_other_game(self):
        other_player = create_game().players.first()

        with self.assertRaises(ValidationError):
            Capital(territory=self.territory, player=other_player).full_clean()

    def test_territory_must_be_owned_by_player(self):
        other_player = self.game.players.exclude(pk=self.player.pk).first()

        with self.assertRaises(ValidationError):
            Capital(territory=self.territory, player=other_player).full_clean()

    def test_health_limits(self):
        for health in [0, 3]:
            with self.subTest(health=health):
                Capital(territory=self.territory, player=self.player, health=health).full_clean()

        for health in [-1, 4]:
            with self.subTest(health=health):
                with self.assertRaises(ValidationError):
                    Capital(territory=self.territory, player=self.player, health=health).full_clean()
