# Quiz Conquest

Triviador style quiz game made with Django REST Framework and React.

## Structure

- `backend/` - Django + Django REST Framework
- `frontend/` - React + Vite

## Backend

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py runserver
```

Load the question bank:

```bash
python manage.py loaddata questions/question_bank.json
```

The fixture has 6 categories, 12 choice questions (with 4 answers each) and 12 numeric questions.

Tests:

```bash
python manage.py test
```

## Frontend

```bash
cd frontend
npm install
npm run dev
```

The app runs on `http://localhost:5173`. Requests to `/api` are sent to the Django server through the Vite proxy, so Django has to be running too.

## Auth API

| Method | Endpoint | Access |
|---|---|---|
| GET | `/api/auth/csrf/` | Public |
| POST | `/api/auth/register/` | Public |
| POST | `/api/auth/login/` | Public |
| POST | `/api/auth/logout/` | Logged in |
| GET | `/api/auth/me/` | Logged in |
| PATCH | `/api/auth/me/` | Logged in |
