/* ============================================================
   Recon-Platform-R2 Web UI — Shared logic
   --------------------------------------------------------------
   - Single SocketIO connection used by every page
   - Health-bar updates (channel + bridge_health + scan state)
   - Wall clock in navbar
   - Keyboard shortcut framework (registered per-page)
   - Toast notifications
   - Small helpers (fmt, esc, timeStr)
   ============================================================ */

const socket = io();

// ---------------------------------------------------------------
// Health bar — one bar across the top of every page
// ---------------------------------------------------------------
//
// SCAN  → /scan rate, comes from bridge_health.frame_counts? no — we
//          infer SCAN state from scan_state events instead (active/paused).
// MAP   → /map channel live + interval. From channel_status.
// POSE  → /scanner/pose channel live. From channel_status.
// IMU   → /imu rate. From channel_status (bridge_health has the rate too).
// ESP32 → bridge_health live + heartbeat freshness.
// LIDAR → motor state from bridge_health.lidar.acked_on.
//
// Each pill shows: dot (color), label, value (rate / ON/OFF / etc).

let scanActiveCached = false;

function setHealth(id, klass, val) {
    const el = document.getElementById(id);
    if (!el) return;
    el.className = 'health-item ' + klass;
    const v = document.getElementById(id + '-val');
    if (v) v.textContent = val;
}

// channel_status: {<name>: {live, last_seen_s}}
socket.on('channel_status', (data) => {
    // MAP — derived from channel staleness
    if (data.map) {
        if (data.map.live)      setHealth('hb-map', 'live', '1 Hz');
        else                    setHealth('hb-map', 'mock', '—');
    }
    if (data.pose) {
        if (data.pose.live)     setHealth('hb-pose', 'live', 'LIVE');
        else                    setHealth('hb-pose', 'mock', 'MOCK');
    }
    if (data.imu) {
        if (data.imu.live)      setHealth('hb-imu', 'live', '100 Hz');
        else                    setHealth('hb-imu', 'mock', '—');
    }
    if (data.bridge_health) {
        if (data.bridge_health.live) setHealth('hb-esp32', 'live', 'LINK');
        else                         setHealth('hb-esp32', 'err',  'DOWN');
    }
    // Dispatch for per-page consumers
    document.dispatchEvent(new CustomEvent('channelStatus', { detail: data }));
});

// scan_state: {active, mode}
socket.on('scan_state', (data) => {
    scanActiveCached = !!data.active;
    if (scanActiveCached) setHealth('hb-scan', 'warn', 'ACTIVE');
    else                  setHealth('hb-scan', 'mock', 'PAUSED');
    document.dispatchEvent(new CustomEvent('scanState', { detail: data }));
});

// bridge_health: rich JSON from /esp32/diagnostics — pluck lidar.acked_on
socket.on('bridge_health', (raw) => {
    try {
        const d = (raw && raw.data) ? JSON.parse(raw.data) : raw;
        if (d && d.lidar) {
            if (d.lidar.acked_on)      setHealth('hb-lidar', 'warn', 'ON');
            else if (d.lidar.desired_on === false) setHealth('hb-lidar', 'mock', 'OFF');
            else                       setHealth('hb-lidar', 'err',  'BAD');
        }
    } catch (e) { /* ignore parse errors */ }
});

// ---------------------------------------------------------------
// Robot events — fan out to per-page listeners
// ---------------------------------------------------------------
socket.on('robot_event', (data) => {
    document.dispatchEvent(new CustomEvent('robotEvent', { detail: data }));
});

socket.on('robot_mode', (data) => {
    document.dispatchEvent(new CustomEvent('robotMode', { detail: data }));
});

// ---------------------------------------------------------------
// Navbar wall clock
// ---------------------------------------------------------------
function tickClock() {
    const el = document.getElementById('navClock');
    if (el) {
        el.textContent = new Date().toLocaleTimeString('en-GB', { hour12: false });
    }
}
setInterval(tickClock, 1000); tickClock();

// ---------------------------------------------------------------
// Keyboard shortcuts
// ---------------------------------------------------------------
const _shortcuts = new Map();  // key → {fn, description}

/** Register a keyboard shortcut.
 *  key: 'Space', 's', 'c', etc. (matches event.code OR event.key.toLowerCase())
 *  fn: handler — receives the KeyboardEvent.
 *  description: shown in any help UI.
 *
 *  Modifier-free only. We deliberately do NOT bind Ctrl/Meta-anything to
 *  avoid colliding with browser/OS chords. */
function registerShortcut(key, fn, description) {
    _shortcuts.set(key.toLowerCase(), { fn, description });
}

document.addEventListener('keydown', (e) => {
    // Ignore when typing
    const t = e.target;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    let key = e.key.toLowerCase();
    if (key === ' ') key = 'space';
    const hit = _shortcuts.get(key);
    if (hit) {
        e.preventDefault();
        hit.fn(e);
    }
});

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------
function fmt(val, decimals = 2) {
    if (val == null) return '—';
    return Number(val).toFixed(decimals);
}

function timeStr() {
    return new Date().toLocaleTimeString('en-GB', { hour12: false });
}

function esc(s) {
    const d = document.createElement('div');
    d.textContent = s;
    return d.innerHTML;
}

// ---------------------------------------------------------------
// Toast
// ---------------------------------------------------------------
function showToast(msg, typeOrSuccess, duration) {
    const type = (typeOrSuccess === true || typeOrSuccess === 'ok') ? 'ok' : 'err';
    const toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.textContent = msg;
    document.body.appendChild(toast);
    requestAnimationFrame(() => toast.classList.add('visible'));
    toast._timerId = setTimeout(() => {
        toast.classList.remove('visible');
        setTimeout(() => toast.remove(), 220);
    }, duration || 3000);
    return toast;
}

function dismissToast(toast) {
    if (!toast) return;
    clearTimeout(toast._timerId);
    toast.classList.remove('visible');
    setTimeout(() => toast.remove(), 220);
}

// ---------------------------------------------------------------
// Mode helpers (back-compat with old per-page code)
// ---------------------------------------------------------------
function setMode(mode) {
    socket.emit('set_mode', { mode: mode });
    syncModeUI(mode);
}

function syncModeUI(mode) {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });
    const modeEl = document.getElementById('current-mode');
    if (modeEl) modeEl.textContent = mode;
}
