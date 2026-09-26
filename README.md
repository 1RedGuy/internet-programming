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
