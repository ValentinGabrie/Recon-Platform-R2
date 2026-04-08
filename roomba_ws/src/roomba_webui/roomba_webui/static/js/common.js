/* ============================================================
   Roomba Web UI — Shared SocketIO + Status Strip Logic
   ============================================================ */

// Global socket connection
const socket = io();

// --- Channel Status Strip (all pages) ---
socket.on('channel_status', (data) => {
    for (const [name, info] of Object.entries(data)) {
        const dot = document.getElementById('dot-' + name);
        if (dot) {
            dot.className = 'dot ' + (info.live ? 'live' : 'mock');
        }
        // Update tooltip
        const ch = dot?.closest('.ch-indicator');
        if (ch) {
            const secs = info.last_seen_s;
            ch.title = info.live
                ? `Live — last ${secs.toFixed(1)}s ago`
                : `Mock — no data for ${secs.toFixed(0)}s`;
        }
    }
    // Dispatch custom event for page-specific handlers
    document.dispatchEvent(new CustomEvent('channelStatus', { detail: data }));
});

// --- Robot Events (all pages) ---
socket.on('robot_event', (data) => {
    document.dispatchEvent(new CustomEvent('robotEvent', { detail: data }));
    // Sync mode display on MODE_CHANGE events
    if (data.type === 'MODE_CHANGE') {
        const mode = data.message.replace('Mode changed to ', '');
        syncModeUI(mode);
    }
});

// --- Robot Mode sync (server pushes at 1 Hz) ---
socket.on('robot_mode', (data) => {
    syncModeUI(data.mode);
});

function syncModeUI(mode) {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });
    const modeEl = document.getElementById('current-mode');
    if (modeEl) modeEl.textContent = mode;
    // Controller page badge
    const badge = document.getElementById('robot-mode');
    if (badge) {
        badge.textContent = mode;
        if (mode === 'MANUAL') badge.className = 'badge badge-connected';
        else if (mode === 'RECON') badge.className = 'badge badge-recon';
        else badge.className = 'badge badge-mock';
    }
}

// --- Mode Helper ---
function setMode(mode) {
    socket.emit('set_mode', { mode: mode });
    syncModeUI(mode);
}

// --- Utility ---
function fmt(val, decimals = 2) {
    if (val == null || val === undefined) return '—';
    return Number(val).toFixed(decimals);
}

function timeStr() {
    return new Date().toLocaleTimeString('en-GB', { hour12: false });
}

// --- HTML Escaping ---
function esc(s) {
    const d = document.createElement('div');
    d.textContent = s;
    return d.innerHTML;
}

// --- Toast Notifications ---
function showToast(msg, typeOrSuccess, duration) {
    const type = (typeOrSuccess === true || typeOrSuccess === 'ok') ? 'ok' : 'err';
    const toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.textContent = msg;
    document.body.appendChild(toast);
    requestAnimationFrame(() => toast.classList.add('visible'));
    toast._timerId = setTimeout(() => {
        toast.classList.remove('visible');
        setTimeout(() => toast.remove(), 300);
    }, duration || 3000);
    return toast;
}

function dismissToast(toast) {
    if (!toast) return;
    clearTimeout(toast._timerId);
    toast.classList.remove('visible');
    setTimeout(() => toast.remove(), 300);
}
