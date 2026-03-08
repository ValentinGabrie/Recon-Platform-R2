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
});

// --- Mode Helper ---
function setMode(mode) {
    socket.emit('set_mode', { mode: mode });
    // Update UI buttons
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });
    const modeEl = document.getElementById('current-mode');
    if (modeEl) modeEl.textContent = mode;
}

// --- Utility ---
function fmt(val, decimals = 2) {
    if (val == null || val === undefined) return '—';
    return Number(val).toFixed(decimals);
}

function timeStr() {
    return new Date().toLocaleTimeString('en-GB', { hour12: false });
}
