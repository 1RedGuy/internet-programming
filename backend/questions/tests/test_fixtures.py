from django.test import TestCase

from questions.models import AnswerOption, Category, ChoiceQuestion, NumericQuestion


class QuestionBankFixtureTests(TestCase):
    fixtures = ["questions/question_bank.json"]

    def test_fixture_is_loaded(self):
        self.assertEqual(Category.objects.count(), 6)
        self.assertEqual(ChoiceQuestion.objects.count(), 12)
        self.assertEqual(AnswerOption.objects.count(), 48)
        self.assertEqual(NumericQuestion.objects.count(), 12)

    def test_choice_questions_are_valid(self):
        for question in ChoiceQuestion.objects.all():
            with self.subTest(question=question.pk):
                self.assertEqual(question.answer_options.count(), 4)
                self.assertEqual(question.answer_options.filter(is_correct=True).count(), 1)
                question.full_clean()

    def test_numeric_questions_are_valid(self):
        for question in NumericQuestion.objects.all():
            with self.subTest(question=question.pk):
                self.assertIsInstance(question.correct_answer, int)
                question.full_clean()
