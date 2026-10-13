import { useState } from 'react'

export function ConnectionSettings({
  url,
  onChange,
}: {
  url: string
  onChange: (url: string) => void
}) {
  const [draft, setDraft] = useState(url)
  const [open, setOpen] = useState(false)

  return (
    <div className="conn-settings">
      <button className="gear-btn" onClick={() => setOpen((o) => !o)} title="Pengaturan koneksi">
        ⚙
      </button>
      {open && (
        <div className="conn-popover">
          <label>rosbridge WebSocket URL</label>
          <input
            value={draft}
            onChange={(e) => setDraft(e.target.value)}
            placeholder="ws://192.168.1.10:9090"
          />
          <button
            className="conn-apply"
            onClick={() => {
              localStorage.setItem('rosbridge_url', draft)
              onChange(draft)
              setOpen(false)
            }}
          >
            Hubungkan
          </button>
        </div>
      )}
    </div>
  )
}
