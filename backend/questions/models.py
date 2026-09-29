from django.core.exceptions import ValidationError
from django.db import models


class Category(models.Model):
    name = models.CharField(max_length=100, unique=True)

    class Meta:
        verbose_name_plural = "categories"

    def __str__(self):
        return self.name


class BaseQuestion(models.Model):
    category = models.ForeignKey(
        Category,
        on_delete=models.PROTECT,
        related_name="%(class)ss",
    )
    text = models.TextField()

    class Meta:
        abstract = True

    def __str__(self):
        return self.text


class ChoiceQuestion(BaseQuestion):
    def clean(self):
        # the answer options are saved after the question,
        # so they can be checked only for a saved question
        if self.pk is None:
            return

        options = self.answer_options.all()
        if options.count() != 4:
            raise ValidationError("A choice question must have exactly 4 answer options.")
        if options.filter(is_correct=True).count() != 1:
            raise ValidationError("A choice question must have exactly one correct answer.")


class NumericQuestion(BaseQuestion):
    correct_answer = models.IntegerField()


class AnswerOption(models.Model):
    question = models.ForeignKey(
        ChoiceQuestion,
        on_delete=models.CASCADE,
        related_name="answer_options",
    )
    text = models.CharField(max_length=255)
    is_correct = models.BooleanField(default=False)

    def __str__(self):
        return self.text
