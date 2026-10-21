from django.conf import settings
from django.core.exceptions import ValidationError
from django.db import models, transaction
from django.db.models.signals import m2m_changed
from django.dispatch import receiver
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

    # the game is locked while players join or leave, so the players
    # can't change while the game is being initialized (games/services.py)
    def add_player(self, user, color):
        with transaction.atomic():
            game = Game.objects.select_for_update().get(pk=self.pk)
            player = Player(game=game, user=user, color=color)
            player.full_clean()
            player.save()
        return player

    def remove_player(self, player):
        with transaction.atomic():
            game = Game.objects.select_for_update().get(pk=self.pk)
            if game.status != "waiting":
                raise ValidationError("Players can leave only a waiting game.")
            if player.game_id != game.pk:
                raise ValidationError("The player is not in this game.")
            player.delete()

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

    def get_capital(self):
        # returns None if the player has no capital yet
        return Capital.objects.filter(player=self).first()

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


class Territory(models.Model):
    game = models.ForeignKey(Game, on_delete=models.CASCADE, related_name="territories")
    name = models.CharField(max_length=50)
    slug = models.SlugField(max_length=50)
    neighbors = models.ManyToManyField("self", blank=True)
    owner = models.ForeignKey(
        Player,
        null=True,
        blank=True,
        on_delete=models.SET_NULL,
        related_name="territories",
    )
    score = models.IntegerField(default=0)

    class Meta:
        verbose_name_plural = "territories"
        constraints = [
            models.UniqueConstraint(
                fields=["game", "name"],
                name="unique_territory_name_per_game",
            ),
            models.UniqueConstraint(
                fields=["game", "slug"],
                name="unique_territory_slug_per_game",
            ),
            models.CheckConstraint(
                condition=models.Q(score__gte=0),
                name="territory_score_gte_0",
            ),
        ]

    def __str__(self):
        return f"{self.name} ({self.game})"

    def clean(self):
        if self.owner_id is not None and self.owner.game_id != self.game_id:
            raise ValidationError({"owner": "The owner must be a player from the same game."})


@receiver(m2m_changed, sender=Territory.neighbors.through)
def check_neighbors(sender, instance, action, pk_set, **kwargs):
    if action != "pre_add":
        return

    if instance.pk in pk_set:
        raise ValidationError("A territory cannot be a neighbor of itself.")
    if Territory.objects.filter(pk__in=pk_set).exclude(game_id=instance.game_id).exists():
        raise ValidationError("Neighbors must be territories from the same game.")


class Capital(models.Model):
    territory = models.OneToOneField(Territory, on_delete=models.CASCADE, related_name="capital")
    player = models.OneToOneField(Player, on_delete=models.CASCADE, related_name="capital")
    health = models.IntegerField(default=3)

    class Meta:
        constraints = [
            models.CheckConstraint(
                condition=models.Q(health__gte=0, health__lte=3),
                name="capital_health_between_0_and_3",
            ),
        ]

    def __str__(self):
        return f"{self.territory.name} - {self.player}"

    @property
    def game(self):
        return self.territory.game

    def clean(self):
        if self.territory_id is None or self.player_id is None:
            return

        if self.territory.game_id != self.player.game_id:
            raise ValidationError("The territory and the player must be from the same game.")
        if self.territory.owner_id != self.player_id:
            raise ValidationError("The capital territory must be owned by the player.")


class WaitingQueue(models.Model):
    STATUS_CHOICES = [
        ("waiting", "Waiting"),
        ("left", "Left"),
        ("matched", "Matched"),
    ]

    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name="waiting_entries")
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default="waiting")
    game = models.ForeignKey(
        Game,
        null=True,
        blank=True,
        on_delete=models.SET_NULL,
        related_name="waiting_entries",
    )
    created_at = models.DateTimeField(auto_now_add=True)
    finished_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=["user"],
                condition=models.Q(status="waiting"),
                name="one_waiting_entry_per_user",
            ),
        ]

    def __str__(self):
        return f"{self.user} ({self.status})"
