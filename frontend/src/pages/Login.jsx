import { useState } from 'react'
import { getErrors, login } from '../api'
import ErrorMessage from '../components/ErrorMessage'

function Login({ onLogin }) {
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [errors, setErrors] = useState({})

  async function handleSubmit(event) {
    event.preventDefault()
    const response = await login(username, password)

    if (response.ok) {
      onLogin(response.data)
    } else {
      setErrors(getErrors(response))
    }
  }

  return (
    <form className="card" onSubmit={handleSubmit}>
      <h2>Login</h2>
      <ErrorMessage messages={errors.non_field_errors} />
      <ErrorMessage messages={errors.detail} />

      <label>
        Username
        <input value={username} onChange={(e) => setUsername(e.target.value)} required />
      </label>
      <ErrorMessage messages={errors.username} />

      <label>
        Password
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
      </label>
      <ErrorMessage messages={errors.password} />

      <button type="submit">Login</button>
    </form>
  )
}

export default Login
