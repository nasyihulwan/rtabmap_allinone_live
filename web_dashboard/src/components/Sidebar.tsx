export type FocusView = 'overview' | 'battery' | 'map'

const ITEMS: { id: FocusView; label: string }[] = [
  { id: 'overview', label: 'Overview' },
  { id: 'battery', label: 'Baterai' },
  { id: 'map', label: 'Peta' },
]

function Icon({ id }: { id: FocusView }) {
  switch (id) {
    case 'overview':
      return (
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
          <path d="M4 11.5 12 4l8 7.5" />
          <path d="M6 10v9a1 1 0 0 0 1 1h3v-5.5h4V20h3a1 1 0 0 0 1-1v-9" />
        </svg>
      )
    case 'battery':
      return (
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
          <rect x="3" y="7" width="16" height="10" rx="2" />
          <path d="M21 10v4" />
          <path d="M7 11.5h4l-1.5 3H13l-3 3" />
        </svg>
      )
    case 'map':
      return (
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
          <path d="M9 4 4 6v14l5-2 6 2 5-2V4l-5 2-6-2Z" />
          <path d="M9 4v14M15 6v14" />
        </svg>
      )
  }
}

export function Sidebar({ active, onSelect }: { active: FocusView; onSelect: (view: FocusView) => void }) {
  return (
    <nav className="sidebar">
      <div className="sidebar-brand" title="Husky A200 Dashboard">HA</div>
      <div className="sidebar-items">
        {ITEMS.map((item) => (
          <button
            key={item.id}
            className={`sidebar-item${active === item.id ? ' active' : ''}`}
            onClick={() => onSelect(item.id)}
            title={item.label}
          >
            <Icon id={item.id} />
          </button>
        ))}
      </div>
    </nav>
  )
}
