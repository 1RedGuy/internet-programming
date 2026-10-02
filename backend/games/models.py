from django.conf import settings
from django.core.exceptions import ValidationError
from django.db import models
from django.utils import timezone


class Game(models.Model):
    STATUS_CHOICES = [
        ("waiting", "Waiting"),
        ("active", "Active"),
        ("completed", "Completed"),
    ]

    created_at = models.DateTimeField(auto_now_add=True)
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default="waiting")

    def __str__(self):
        return f"Game #{self.pk}"

    def get_current_round(self):
        return self.rounds.order_by("-number").first()

    def is_active(self):
        return self.status == "active"

    def is_completed(self):
        return self.status == "completed"

    def add_player(self, user, color):
        player = Player(game=self, user=user, color=color)
        player.full_clean()
        player.save()
        return player

    def start(self):
        if self.status != "waiting":
            raise ValidationError("Only a waiting game can be started.")
        if self.players.count() != 3:
            raise ValidationError("A game needs exactly 3 players to start.")

        self.status = "active"
        self.save()

    def complete(self):
        if self.status != "active":
            raise ValidationError("Only an active game can be completed.")

        self.status = "completed"
        self.save()

    def create_round(self, round_type):
        last_round = self.get_current_round()
        if last_round and last_round.status != "completed":
            raise ValidationError("The current round is not completed yet.")

        number = last_round.number + 1 if last_round else 1
        game_round = Round(game=self, number=number, type=round_type)
        game_round.full_clean()
        game_round.save()
        return game_round


class Player(models.Model):
    COLOR_CHOICES = [
        ("red", "Red"),
        ("green", "Green"),
        ("blue", "Blue"),
    ]

    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE)
    game = models.ForeignKey(Game, on_delete=models.CASCADE, related_name="players")
    score = models.IntegerField(default=0)
    color = models.CharField(max_length=10, choices=COLOR_CHOICES)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=["user", "game"],
                name="unique_user_per_game",
            ),
            models.UniqueConstraint(
                fields=["game", "color"],
                name="unique_color_per_game",
            ),
            models.CheckConstraint(
                condition=models.Q(score__gte=0),
                name="player_score_gte_0",
            ),
        ]

    def __str__(self):
        return f"{self.user} ({self.color})"

    def clean(self):
        # checks only for new players joining a game
        if self.pk is None and self.game_id is not None:
            if self.game.status != "waiting":
                raise ValidationError("Players can join only a waiting game.")
            if self.game.players.count() >= 3:
                raise ValidationError("A game can have at most 3 players.")


class Round(models.Model):
    TYPE_CHOICES = [
        ("city_capture", "City capture"),
        ("battle", "Battle"),
        ("capital_attack", "Capital attack"),
        ("bonus", "Bonus"),
    ]
    STATUS_CHOICES = [
        ("pending", "Pending"),
        ("active", "Active"),
        ("completed", "Completed"),
    ]

    game = models.ForeignKey(Game, on_delete=models.CASCADE, related_name="rounds")
    number = models.PositiveIntegerField()
    type = models.CharField(max_length=20, choices=TYPE_CHOICES)
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default="pending")
    winner = models.ForeignKey(
        Player,
        null=True,
        blank=True,
        on_delete=models.SET_NULL,
        related_name="won_rounds",
    )
    created_at = models.DateTimeField(auto_now_add=True)
    completed_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=["game", "number"],
                name="unique_round_number_per_game",
            ),
            models.UniqueConstraint(
                fields=["game"],
                condition=models.Q(status="active"),
                name="one_active_round_per_game",
            ),
        ]

    def __str__(self):
        return f"Round {self.number} ({self.type}) - {self.game}"

    def clean(self):
        if self.pk is None and self.game_id is not None and self.game.status != "active":
            raise ValidationError("Rounds can be created only in an active game.")
        if self.winner_id is not None and self.winner.game_id != self.game_id:
            raise ValidationError({"winner": "The winner must be a player from the same game."})
        if self.status == "completed" and (self.winner_id is None or self.completed_at is None):
            raise ValidationError("A completed round must have a winner and completed_at.")

    def start(self):
        if self.status != "pending":
            raise ValidationError("Only a pending round can be started.")

        self.status = "active"
        self.full_clean()
        self.save()

    def complete(self, winner):
        if self.status != "active":
            raise ValidationError("Only an active round can be completed.")

        self.winner = winner
        self.status = "completed"
        self.completed_at = timezone.now()
        self.full_clean()
        self.save()
