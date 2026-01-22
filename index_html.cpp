#include <pgmspace.h>  // needed for PROGMEM

// Definition of HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>ESP32 Stepper Control</title>
  <style>
    body { font-family: Arial; margin: 12px; }
    label { display: block; margin-top: 8px; }
    input[type=number] { width: 140px; transition: background 0.3s, color 0.3s; }
    button { margin-top: 8px; padding: 6px 12px; transition: background 0.2s; }
    button:active { background: #ddd; }
    .row { display: flex; gap: 8px; align-items: center; }
    .status { margin-top: 12px; padding: 8px; border: 1px solid #ccc; border-radius: 6px; }
    .spinner {
      width: 48px; height: 48px;
      border: 6px solid rgba(0,0,0,0.2);
      border-top: 6px solid #3498db;
      border-radius: 50%;
      animation: spin 1s linear infinite;
      margin: 10px auto 0;
    }
    @keyframes spin { 0% {transform: rotate(0deg);} 100% {transform: rotate(360deg);} }
    /* Feedback states */
    .edited  { background: #fff3cd; color: black; }   /* yellow */
    .pending { background: #ffe0b2; color: black; }   /* orange */
    .applied { background: #c8e6c9; color: #1b5e20; } /* green bg + dark green text */
    #queuedNotice { color: orange; font-size: 0.9em; display: none; margin-top: 6px; }
  </style>
</head>
<body>
  <h3>ESP32 Stepper Control</h3>
  <div>
    <label>Speed: <input id="speed" type="number" /></label>
    <label>Accel: <input id="accel" type="number" /></label>
    <label>Decel: <input id="decel" type="number" /></label>
    <label>Distance: <input id="distance" type="number" /></label>
    <label>Delay: <input id="delay" type="number" /></label>
    <label><input type="checkbox" id="direction" /> Direction (CW)</label>
    <div class="row">
      <button onclick="applyParams(this)">Apply</button>
      <button onclick="run(1,this)">Run</button>
      <button onclick="run(0,this)">Stop</button>
      <button onclick="handshake(this)">Handshake</button>
    </div>
    <div id="queuedNotice">Parameters will apply on next Run</div>
  </div>
  <div class="status">
    <div>Running: <span id="running">-</span></div>
    <div>Dir: <span id="dir">-</span></div>
    <div>Temp: <span id="temp">-</span> °C</div>
    <div>Humidity: <span id="humid">-</span> %</div>
    <div>Speed (live): <span id="lspeed">-</span></div>
    <div>Distance (live): <span id="ldistance">-</span></div>
    <div>MAC: <span id="mac" class="pending">-</span></div>
    <div>Channel: <span id="chan" class="pending">-</span></div>
  </div>
  <div id="motorSpinner" style="position: fixed; bottom: 16px; width: 100%; text-align: center;"></div>

  <script>
    const inputs = ['speed','accel','decel','distance','delay'];
    let lastAck = { speed:0, accel:0, decel:0, distance:0, delay:0, direction:0 };

    // Mark edits yellow
    inputs.forEach(id => {
      const el = document.getElementById(id);
      el.addEventListener('input', () => {
        el.classList.add('edited');
        el.classList.remove('applied','pending');
        document.getElementById('queuedNotice').style.display = 'block';
      });
    });
    document.getElementById('direction').addEventListener('change', () => {
      const el = document.getElementById('direction');
      el.classList.add('edited');
      el.classList.remove('applied','pending');
      document.getElementById('queuedNotice').style.display = 'block';
    });

    async function apiGet(path) {
      const res = await fetch(path);
      return res.ok ? res.json() : null;
    }

    async function poll() {
      const j = await apiGet('/api/live');
      if (!j) return;

      // live status
      const running = j.running;
      document.getElementById('running').textContent = running ? 'YES' : 'NO';
      document.getElementById('motorSpinner').innerHTML = running ? '<div class="spinner"></div>' : '';
      document.getElementById('dir').textContent = j.params.direction ? 'CW' : 'CCW';
      document.getElementById('temp').textContent = j.temp.toFixed(1);
      document.getElementById('humid').textContent = j.humid.toFixed(1);
      document.getElementById('lspeed').textContent = j.speed;
      document.getElementById('ldistance').textContent = j.distance;

      // MAC & Channel
      document.getElementById('mac').textContent = j.mac;
      document.getElementById('mac').className = 'applied';
      document.getElementById('chan').textContent = j.channel;
      document.getElementById('chan').className = 'applied';

      // update inputs with ACK
      inputs.forEach(id => {
        const el = document.getElementById(id);
        const newVal = j.params[id];
        if (document.activeElement !== el) el.value = newVal;
        if (parseInt(el.value) === parseInt(newVal)) {
          el.className = 'applied';
          lastAck[id] = newVal;
        } else {
          if (!el.classList.contains('edited')) el.className = '';
        }
      });

      // direction checkbox
      const dirEl = document.getElementById('direction');
      if (document.activeElement !== dirEl) dirEl.checked = !!j.params.direction;
      if ((dirEl.checked ? 1:0) === (j.params.direction ? 1:0)) {
        dirEl.className = 'applied';
        lastAck.direction = j.params.direction ? 1:0;
      } else {
        if (!dirEl.classList.contains('edited')) dirEl.className = '';
      }

      // hide queued notice if all applied
      const anyEdited = [...inputs, 'direction'].some(id => {
        const el = (id==='direction') ? dirEl : document.getElementById(id);
        return el.classList.contains('edited');
      });
      document.getElementById('queuedNotice').style.display = anyEdited ? 'block' : 'none';

      // disable Apply while running
      document.querySelector('button[onclick^="applyParams"]').disabled = running;
    }

    async function applyParams(btn) {
      const q = new URLSearchParams();
      inputs.forEach(id => {
        const el = document.getElementById(id);
        q.append(id, el.value);
        el.classList.remove('edited');
        el.classList.add('pending');
      });
      const dirEl = document.getElementById('direction');
      q.append('direction', dirEl.checked ? '1':'0');
      dirEl.classList.remove('edited');
      dirEl.classList.add('pending');

      btn.style.background="#90caf9";
      await fetch('/api/set?' + q.toString());
      setTimeout(()=>btn.style.background="",300);
    }

    async function run(v,btn) {
      btn.style.background = "#90caf9";
      await fetch('/run?val=' + v);
      setTimeout(() => btn.style.background = "", 300);
    }

    async function handshake(btn) {
      btn.style.background = "#90caf9";
      await fetch('/handshake');
      setTimeout(() => btn.style.background = "", 300);
    }

    setInterval(poll, 500);
    window.onload = poll;
  </script>
</body>
</html>
)rawliteral";
