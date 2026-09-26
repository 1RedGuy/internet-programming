import { useEffect, useState } from 'react'
import { getCsrfToken, getMe, logout } from './api'
import Login from './pages/Login'
import Register from './pages/Register'
import Profile from './pages/Profile'
import './App.css'

function App() {
  const [user, setUser] = useState(null)
  const [page, setPage] = useState('login')
  const [loading, setLoading] = useState(true)

  // check if there is already a session (for example after refresh)
  useEffect(() => {
    async function loadUser() {
      await getCsrfToken()
      const response = await getMe()
      if (response.ok) {
        setUser(response.data)
      }
      setLoading(false)
    }
    loadUser()
  }, [])

  async function handleLogout() {
    await logout()
    setUser(null)
    setPage('login')
  }

  if (loading) {
    return <p className="loading">Loading...</p>
  }

  return (
    <div className="app">
      <header className="header">
        <h1>Quiz Conquest</h1>
        <nav>
          {user ? (
            <button onClick={handleLogout}>Logout</button>
          ) : (
            <>
              <button className={page === 'login' ? 'active' : ''} onClick={() => setPage('login')}>
                Login
              </button>
              <button className={page === 'register' ? 'active' : ''} onClick={() => setPage('register')}>
                Register
              </button>
            </>
          )}
        </nav>
      </header>

      <main>
        {user && <Profile user={user} onUpdate={setUser} />}
        {!user && page === 'login' && <Login onLogin={setUser} />}
        {!user && page === 'register' && <Register onRegister={setUser} />}
      </main>
    </div>
  )
}

export default App
