from django.contrib import admin
from django.core.exceptions import ValidationError
from django.forms.models import BaseInlineFormSet

from .models import AnswerOption, Category, ChoiceQuestion, NumericQuestion


class AnswerOptionFormSet(BaseInlineFormSet):
    def clean(self):
        super().clean()

        answers = 0
        correct_answers = 0
        for form in self.forms:
            if not form.cleaned_data or form.cleaned_data.get("DELETE"):
                continue
            answers += 1
            if form.cleaned_data.get("is_correct"):
                correct_answers += 1

        if answers != 4:
            raise ValidationError("A choice question must have exactly 4 answer options.")
        if correct_answers != 1:
            raise ValidationError("Exactly one answer option must be correct.")


class AnswerOptionInline(admin.TabularInline):
    model = AnswerOption
    formset = AnswerOptionFormSet
    extra = 0
    min_num = 4
    max_num = 4


@admin.register(Category)
class CategoryAdmin(admin.ModelAdmin):
    list_display = ["name"]
    search_fields = ["name"]


@admin.register(ChoiceQuestion)
class ChoiceQuestionAdmin(admin.ModelAdmin):
    list_display = ["text", "category"]
    list_filter = ["category"]
    search_fields = ["text"]
    inlines = [AnswerOptionInline]


@admin.register(NumericQuestion)
class NumericQuestionAdmin(admin.ModelAdmin):
    list_display = ["text", "category", "correct_answer"]
    list_filter = ["category"]
    search_fields = ["text"]
