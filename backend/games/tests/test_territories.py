from django.core.exceptions import ValidationError
from django.db import IntegrityError, transaction
from django.test import TestCase

from games.game_map import NEIGHBORS, TERRITORIES, validate_game_map
from games.models import Capital, Game, Territory

from .utils import create_game, create_map


class TerritoryTests(TestCase):
    def setUp(self):
        self.game = create_game()
        self.player = self.game.players.first()

    def test_game_map_matches_definition(self):
        territories = create_map(self.game)

        validate_game_map(self.game)
        self.assertEqual(self.game.territories.count(), len(TERRITORIES))
        self.assertEqual(
            {neighbor.slug for neighbor in territories["madara"].neighbors.all()},
            set(NEIGHBORS["madara"]),
        )

    def test_game_map_with_wrong_data(self):
        def wrong_name(territories):
            territories["skali"].name = "Нещо друго"
            territories["skali"].save()

        def missing_neighbor(territories):
            territories["skali"].neighbors.remove(territories["dunaviya"])

        def extra_neighbor(territories):
            territories["skali"].neighbors.add(territories["tsarevo"])

        def missing_territory(territories):
            territories["tsarevo"].delete()

        for change in [wrong_name, missing_neighbor, extra_neighbor, missing_territory]:
            with self.subTest(change.__name__):
                game = create_game()
                change(create_map(game))

                with self.assertRaises(ValidationError):
                    validate_game_map(game)

    def test_neighbors_are_symmetric(self):
        first = Territory.objects.create(game=self.game, name="Скали", slug="skali")
        second = Territory.objects.create(game=self.game, name="Дунавия", slug="dunaviya")

        first.neighbors.add(second)

        self.assertIn(first, second.neighbors.all())

    def test_invalid_neighbors(self):
        territory = Territory.objects.create(game=self.game, name="Скали", slug="skali")
        other_game_territory = Territory.objects.create(game=create_game(), name="Дунавия", slug="dunaviya")

        with self.assertRaises(ValidationError):
            with transaction.atomic():
                territory.neighbors.add(territory)
        with self.assertRaises(ValidationError):
            with transaction.atomic():
                territory.neighbors.add(other_game_territory)

        self.assertEqual(territory.neighbors.count(), 0)

    def test_name_and_slug_are_unique_in_game(self):
        Territory.objects.create(game=self.game, name="Скали", slug="skali")

        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Territory.objects.create(game=self.game, name="Скали", slug="other")
        with self.assertRaises(IntegrityError):
            with transaction.atomic():
                Territory.objects.create(game=self.game, name="Друго", slug="skali")

        Territory.objects.create(game=create_game(), name="Скали", slug="skali")

    def test_owner(self):
        territory = Territory(game=self.game, name="Скали", slug="skali")
        territory.full_clean()

        territory.owner = self.player
        territory.full_clean()

        territory.owner = create_game().players.first()
        with self.assertRaises(ValidationError):
            territory.full_clean()

    def test_score_cannot_be_negative(self):
        territory = Territory(game=self.game, name="Скали", slug="skali", score=-1)

        with self.assertRaises(ValidationError):
            territory.full_clean()
        with self.assertRaises(IntegrityError):
            territory.save()

    def test_games_are_independent(self):
        other_game = create_game()
        territories = create_map(self.game)
        other_territories = create_map(other_game)

        territories["sredets"].owner = self.player
        territories["sredets"].score = 200
        territories["sredets"].save()

        other_territories["sredets"].refresh_from_db()
        self.assertIsNone(other_territories["sredets"].owner)
        self.assertEqual(other_territories["sredets"].score, 0)
        self.assertNotEqual(territories["sredets"].pk, other_territories["sredets"].pk)
        for territory in territories.values():
            self.assertFalse(territory.neighbors.filter(game=other_game).exists())

    def test_player_territories_and_capital(self):
        territories = create_map(self.game)
        self.assertIsNone(self.player.get_capital())

        territories["skali"].owner = self.player
        territories["skali"].save()
        territories["pirina"].owner = self.player
        territories["pirina"].save()
        capital = Capital.objects.create(territory=territories["skali"], player=self.player)

        self.assertEqual(set(self.player.territories.all()), {territories["skali"], territories["pirina"]})
        self.assertEqual(self.player.get_capital(), capital)

    def test_creating_game_does_not_create_map(self):
        game = Game.objects.create()

        self.assertEqual(game.status, "waiting")
        self.assertEqual(game.territories.count(), 0)
        self.assertFalse(Capital.objects.filter(territory__game=game).exists())
