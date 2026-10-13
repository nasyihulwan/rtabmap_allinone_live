import { useState } from 'react'
import { useRosConnection } from './hooks/useRosConnection'
import { useNav2Goal } from './hooks/useNav2Goal'
import { useMapData } from './lib/mapData'
import { RobotViewer } from './components/RobotViewer'
import { TelemetryPanel } from './components/TelemetryPanel'
import { ConnectionSettings } from './components/ConnectionSettings'
import { Sidebar, type FocusView } from './components/Sidebar'
import { BatteryPanel } from './components/BatteryPanel'
import { MapPanel, OverviewMapPane } from './components/MapPanel'
import { ErrorBoundary } from './components/ErrorBoundary'
import './App.css'

const DEFAULT_URL = localStorage.getItem('rosbridge_url') || 'ws://192.168.1.10:9090'
const FALLBACK_POSE = { x: 0, y: 0, z: 0, yaw: 0 }

function App() {
  const [url, setUrl] = useState(DEFAULT_URL)
  // Telemetri (pose, baterai, obstacle person/lab-robot) DAN navigasi
  // sekarang sama-sama didorong data riil: pose/baterai/obstacle dari `ros`
  // (rosbridge topic), goal "navigate to object" dari `nav` (ROSLIB.Action
  // ke /navigate_to_pose asli, path digambar dari /plan asli — lihat
  // useNav2Goal.ts). Sebelumnya navigasi pakai pathfinder lokal palsu di
  // useSimEngine.ts (masih ada di repo, sudah tidak dipakai di sini).
  const ros = useRosConnection(url)

  const { data: mapData } = useMapData()
  const navHook = useNav2Goal(
    ros.rosInstance,
    mapData?.objects ?? [],
    ros.speed,
    mapData ? { meta: mapData.meta, grid: mapData.grid } : null,
  )

  const [focus, setFocus] = useState<FocusView>('overview')
  const [mapVisible, setMapVisible] = useState(true)
  const [showZones, setShowZones] = useState(true)
  const [showObjects, setShowObjects] = useState(true)

  const pose = ros.pose ?? FALLBACK_POSE
  const moving = navHook.nav.status === 'moving'

  return (
    <div className="dashboard">
      <Sidebar active={focus} onSelect={setFocus} />

      <div className="main-area">
        <ConnectionSettings url={url} onChange={setUrl} />
        <div className="ros-chip" title="Status koneksi rosbridge — pose/baterai/obstacle di dashboard ini datang dari sini">
          <span className={`status-dot status-dot-${ros.status}`} />
          rosbridge: {ros.status}
        </div>

        {false && (
          <div className="sim-banner">
            Telemetri dan navigasi robot riil — semua data (pose/baterai/obstacle/path) dari rosbridge
          </div>
        )}

        <div className="view-layer" data-active={focus === 'overview'}>
          <div className="overview-split">
            <div className="robot-pane">
              <RobotViewer pose={ros.pose} speed={ros.speed} obstacles={ros.obstacles} />
              <TelemetryPanel
                statusLabel={`rosbridge ${ros.status}`}
                statusKind={ros.status}
                pose={ros.pose}
                batteryPercentage={ros.battery.percentage ?? 0}
                batteryVoltage={ros.battery.voltage ?? 0}
                batteryHistory={ros.battery.history}
                speed={ros.speed}
                obstacles={ros.obstacles}
              />
              {!mapVisible && (
                <button className="show-map-btn" onClick={() => setMapVisible(true)}>
                  Tampilkan peta
                </button>
              )}
            </div>

            {mapData && mapVisible && (
              <div className="map-pane">
                <ErrorBoundary>
                  <OverviewMapPane
                    meta={mapData.meta}
                    objects={mapData.objects}
                    pose={pose}
                    moving={moving}
                    nav={navHook.nav}
                    distanceRemaining={navHook.distanceRemaining}
                    etaSeconds={navHook.etaSeconds}
                    onNavigate={navHook.navigateTo}
                    onCancel={navHook.cancelNavigation}
                    showZones={showZones}
                    showObjects={showObjects}
                    onToggleZones={() => setShowZones((v) => !v)}
                    onToggleObjects={() => setShowObjects((v) => !v)}
                    onExpand={() => setFocus('map')}
                    onHide={() => setMapVisible(false)}
                  />
                </ErrorBoundary>
              </div>
            )}
          </div>
        </div>

        <div className="view-layer" data-active={focus === 'battery'}>
          <BatteryPanel
            battery={{
              percentage: ros.battery.percentage ?? 0,
              voltage: ros.battery.voltage ?? 0,
              current: ros.battery.current ?? 0,
              history: ros.battery.history,
            }}
            moving={moving}
          />
        </div>

        <div className="view-layer" data-active={focus === 'map'}>
          <ErrorBoundary>
            {mapData ? (
              <MapPanel
                meta={mapData.meta}
                objects={mapData.objects}
                pose={pose}
                moving={moving}
                nav={navHook.nav}
                distanceRemaining={navHook.distanceRemaining}
                etaSeconds={navHook.etaSeconds}
                onNavigate={navHook.navigateTo}
                onCancel={navHook.cancelNavigation}
                showZones={showZones}
                showObjects={showObjects}
                onToggleZones={() => setShowZones((v) => !v)}
                onToggleObjects={() => setShowObjects((v) => !v)}
              />
            ) : (
              <div className="panel-detail map-loading">Memuat data peta…</div>
            )}
          </ErrorBoundary>
        </div>
      </div>
    </div>
  )
}

export default App
