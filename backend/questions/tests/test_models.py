from django.core.exceptions import ValidationError
from django.db import IntegrityError
from django.db.models import ProtectedError
from django.test import TestCase

from questions.models import AnswerOption, Category, ChoiceQuestion, NumericQuestion


class CategoryTests(TestCase):
    def test_create_category(self):
        category = Category.objects.create(name="География")

        category.full_clean()
        self.assertEqual(str(category), "География")

    def test_name_is_unique(self):
        Category.objects.create(name="География")

        with self.assertRaises(IntegrityError):
            Category.objects.create(name="География")

    def test_category_with_questions_cannot_be_deleted(self):
        category = Category.objects.create(name="География")
        NumericQuestion.objects.create(category=category, text="Колко области има България?", correct_answer=28)

        with self.assertRaises(ProtectedError):
            category.delete()
        self.assertTrue(Category.objects.filter(pk=category.pk).exists())


class ChoiceQuestionTests(TestCase):
    def setUp(self):
        self.category = Category.objects.create(name="Наука")
        self.question = ChoiceQuestion.objects.create(
            category=self.category,
            text="Кой е химичният символ на златото?",
        )

    def add_options(self, correct_flags):
        for number, is_correct in enumerate(correct_flags, start=1):
            AnswerOption.objects.create(
                question=self.question,
                text=f"Отговор {number}",
                is_correct=is_correct,
            )

    def test_valid_choice_question(self):
        self.add_options([True, False, False, False])

        self.question.full_clean()
        self.assertEqual(self.question.category, self.category)
        self.assertEqual(self.question.answer_options.count(), 4)
        self.assertEqual(self.question.answer_options.filter(is_correct=True).count(), 1)

    def test_invalid_choice_question(self):
        cases = {
            "three options": [True, False, False],
            "five options": [True, False, False, False, False],
            "no correct answer": [False, False, False, False],
            "two correct answers": [True, True, False, False],
        }

        for name, correct_flags in cases.items():
            with self.subTest(name):
                self.question.answer_options.all().delete()
                self.add_options(correct_flags)

                with self.assertRaises(ValidationError):
                    self.question.full_clean()

    def test_deleting_question_deletes_answer_options(self):
        self.add_options([True, False, False, False])

        self.question.delete()

        self.assertEqual(AnswerOption.objects.count(), 0)


class NumericQuestionTests(TestCase):
    def setUp(self):
        self.category = Category.objects.create(name="История")

    def test_create_numeric_question(self):
        question = NumericQuestion.objects.create(
            category=self.category,
            text="През коя година е основана Дунавска България?",
            correct_answer=681,
        )

        question.full_clean()
        self.assertEqual(question.correct_answer, 681)
        self.assertEqual(list(self.category.numericquestions.all()), [question])

    def test_correct_answer_is_required(self):
        question = NumericQuestion(category=self.category, text="Въпрос без отговор")

        with self.assertRaises(ValidationError):
            question.full_clean()
