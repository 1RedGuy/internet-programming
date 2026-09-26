import { useState } from 'react'
import { getErrors, updateMe } from '../api'
import ErrorMessage from '../components/ErrorMessage'

const avatars = ['knight-1', 'knight-2', 'knight-3', 'knight-4']

function Profile({ user, onUpdate }) {
  const [nickname, setNickname] = useState(user.profile.nickname)
  const [avatarKey, setAvatarKey] = useState(user.profile.avatar_key)
  const [errors, setErrors] = useState({})
  const [message, setMessage] = useState('')

  async function handleSubmit(event) {
    event.preventDefault()
    setMessage('')
    const response = await updateMe({ nickname, avatar_key: avatarKey })

    if (response.ok) {
      setErrors({})
      setMessage('Profile saved.')
      onUpdate(response.data)
    } else {
      setErrors(getErrors(response))
    }
  }

  return (
    <div className="card">
      <h2>Profile</h2>
      <div className="user-info">
        <div className={`avatar ${user.profile.avatar_key}`}>
          {user.profile.avatar_key.split('-')[1]}
        </div>
        <div>
          <p><b>{user.profile.nickname}</b></p>
          <p>Username: {user.username}</p>
          <p>Email: {user.email}</p>
        </div>
      </div>

      <form onSubmit={handleSubmit}>
        <ErrorMessage messages={errors.detail} />

        <label>
          Nickname
          <input value={nickname} onChange={(e) => setNickname(e.target.value)} maxLength={30} required />
        </label>
        <ErrorMessage messages={errors.nickname} />

        <p>Avatar</p>
        <div className="avatar-list">
          {avatars.map((avatar) => (
            <button
              type="button"
              key={avatar}
              className={`avatar ${avatar} ${avatar === avatarKey ? 'selected' : ''}`}
              onClick={() => setAvatarKey(avatar)}
            >
              {avatar.split('-')[1]}
            </button>
          ))}
        </div>
        <ErrorMessage messages={errors.avatar_key} />

        {message && <p className="success">{message}</p>}
        <button type="submit">Save</button>
      </form>
    </div>
  )
}

export default Profile
