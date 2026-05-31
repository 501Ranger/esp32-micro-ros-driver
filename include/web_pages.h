#pragma once

#include <Arduino.h>

namespace robot {

const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32 机器人控制终端</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@400;600;800&family=Inter:wght@400;500;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-color: #080c14;
      --card-bg: rgba(18, 24, 38, 0.6);
      --card-border: rgba(255, 255, 255, 0.08);
      --accent-color: #6366f1;
      --accent-gradient: linear-gradient(135deg, #6366f1, #a855f7);
      --text-main: #f3f4f6;
      --text-muted: #9ca3af;
      --success: #10b981;
      --error: #ef4444;
      --estop-yellow: #f59e0b;
    }

    * {
      box-sizing: border-box;
      user-select: none;
      -webkit-user-select: none;
    }

    body {
      margin: 0;
      padding: 0;
      background-color: var(--bg-color);
      background-image: 
      	radial-gradient(at 0% 0%, rgba(99, 102, 241, 0.15) 0px, transparent 50%),
      	radial-gradient(at 100% 100%, rgba(168, 85, 247, 0.15) 0px, transparent 50%);
      color: var(--text-main);
      font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
      overflow: hidden;
      touch-action: none;
      display: flex;
      flex-direction: column;
      height: 100vh;
      height: 100dvh;
      width: 100vw;
    }

    header {
      width: 100%;
      padding: 14px 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: rgba(8, 12, 20, 0.8);
      backdrop-filter: blur(10px);
      border-bottom: 1px solid var(--card-border);
      z-index: 10;
    }

    .logo-container {
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .logo-text {
      font-family: 'Outfit', sans-serif;
      font-weight: 800;
      font-size: 18px;
      background: var(--accent-gradient);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      letter-spacing: -0.5px;
    }

    .header-actions {
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .btn-nav-header {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      padding: 6px 12px;
      border-radius: 9999px;
      background: rgba(99, 102, 241, 0.12);
      border: 1px solid rgba(99, 102, 241, 0.3);
      color: #a5b4fc;
      font-size: 13px;
      font-weight: 600;
      text-decoration: none;
      transition: all 0.2s ease;
    }

    .btn-nav-header:active {
      background: rgba(99, 102, 241, 0.25);
      border-color: var(--accent-color);
    }

    .status-badge {
      display: flex;
      align-items: center;
      gap: 6px;
      padding: 6px 12px;
      border-radius: 9999px;
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid var(--card-border);
      font-size: 13px;
      font-weight: 500;
      transition: all 0.3s ease;
    }

    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background-color: var(--error);
      box-shadow: 0 0 8px var(--error);
    }

    .status-badge.connected .status-dot {
      background-color: var(--success);
      box-shadow: 0 0 8px var(--success);
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0% { opacity: 0.6; }
      50% { opacity: 1; }
      100% { opacity: 0.6; }
    }

    main {
      flex: 1;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: space-around;
      padding: 16px;
      gap: 16px;
      max-width: 500px;
      margin: 0 auto;
      width: 100%;
    }

    .dashboard-panel {
      width: 100%;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }

    .stat-card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      backdrop-filter: blur(12px);
      border-radius: 16px;
      padding: 12px 14px;
      display: flex;
      align-items: center;
      gap: 10px;
      box-shadow: 0 4px 20px rgba(0,0,0,0.15);
    }

    .stat-icon {
      width: 36px;
      height: 36px;
      border-radius: 8px;
      background: rgba(99, 102, 241, 0.1);
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--accent-color);
    }

    .stat-content {
      display: flex;
      flex-direction: column;
    }

    .stat-label {
      font-size: 10px;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin-bottom: 2px;
    }

    .stat-value {
      font-size: 14px;
      font-weight: 600;
      font-family: 'Outfit', sans-serif;
    }

    /* Switch styles */
    .switch {
      position: relative;
      display: inline-block;
      width: 44px;
      height: 24px;
    }
    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }
    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: rgba(255, 255, 255, 0.1);
      transition: .4s;
      border: 1px solid var(--card-border);
    }
    .slider:before {
      position: absolute;
      content: "";
      height: 16px;
      width: 16px;
      left: 3px;
      bottom: 3px;
      background-color: var(--text-muted);
      transition: .4s;
    }
    input:checked + .slider {
      background-color: var(--accent-color);
      border-color: var(--accent-color);
    }
    input:checked + .slider:before {
      transform: translateX(20px);
      background-color: white;
    }
    .slider.round {
      border-radius: 24px;
    }
    .slider.round:before {
      border-radius: 50%;
    }



    .joystick-container {
      flex: 1;
      width: 100%;
      display: flex;
      align-items: center;
      justify-content: center;
      position: relative;
    }

    .joystick-zone {
      position: relative;
      width: 280px;
      height: 280px;
      background: rgba(18, 24, 38, 0.4);
      border: 2px solid var(--card-border);
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 
        inset 0 0 30px rgba(0,0,0,0.6),
        0 8px 24px rgba(0,0,0,0.3);
      backdrop-filter: blur(8px);
      transition: border-color 0.3s ease;
    }

    .joystick-zone::before {
      content: '';
      position: absolute;
      width: 100%;
      height: 100%;
      border-radius: 50%;
      background: 
        radial-gradient(circle, transparent 35%, rgba(99, 102, 241, 0.03) 36%, rgba(99, 102, 241, 0.03) 38%, transparent 39%),
        radial-gradient(circle, transparent 65%, rgba(99, 102, 241, 0.03) 66%, rgba(99, 102, 241, 0.03) 68%, transparent 69%);
      pointer-events: none;
    }

    .joystick-axis-h, .joystick-axis-v {
      position: absolute;
      background: rgba(255, 255, 255, 0.04);
      pointer-events: none;
    }
    .joystick-axis-h { width: 80%; height: 1px; }
    .joystick-axis-v { height: 80%; width: 1px; }

    .stick {
      position: absolute;
      width: 90px;
      height: 90px;
      background: radial-gradient(circle at 30% 30%, #374151 0%, #111827 100%);
      border: 2px solid #4b5563;
      border-radius: 50%;
      box-shadow: 
        0 10px 20px rgba(0,0,0,0.6),
        inset 0 2px 4px rgba(255,255,255,0.15);
      pointer-events: none;
      transition: transform 0.15s cubic-bezier(0.25, 0.8, 0.25, 1);
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .stick::after {
      content: '';
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: var(--accent-gradient);
      box-shadow: 0 0 10px var(--accent-color);
      opacity: 0.8;
      transition: opacity 0.3s ease;
    }

    .stick.active {
      transition: none;
      border-color: var(--accent-color);
      box-shadow: 
        0 12px 24px rgba(99, 102, 241, 0.3),
        inset 0 2px 4px rgba(255,255,255,0.2);
    }

    /* Custom Slider Styling */
    .custom-slider {
      -webkit-appearance: none;
      appearance: none;
      width: 100%;
      height: 10px;
      border-radius: 5px;
      background: linear-gradient(to right, #6366f1 0%, #a855f7 var(--value, 61.5%), rgba(255, 255, 255, 0.15) var(--value, 61.5%), rgba(255, 255, 255, 0.15) 100%);
      outline: none;
      transition: background 0.1s ease;
      cursor: pointer;
    }

    .custom-slider:hover {
      background: linear-gradient(to right, #6366f1 0%, #a855f7 var(--value, 61.5%), rgba(255, 255, 255, 0.2) var(--value, 61.5%), rgba(255, 255, 255, 0.2) 100%);
    }

    .custom-slider::-webkit-slider-runnable-track {
      width: 100%;
      height: 10px;
      border-radius: 5px;
      background: transparent;
    }

    .custom-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: var(--accent-gradient);
      box-shadow: 0 0 10px var(--accent-color);
      cursor: pointer;
      margin-top: -6px; /* (10px track height - 22px thumb height) / 2 = -6px */
      transition: transform 0.15s cubic-bezier(0.34, 1.56, 0.64, 1), box-shadow 0.15s ease;
    }

    .custom-slider::-webkit-slider-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 0 16px var(--accent-color), 0 0 4px rgba(255, 255, 255, 0.4);
    }

    .custom-slider::-webkit-slider-thumb:active {
      transform: scale(1.4);
      box-shadow: 0 0 24px var(--accent-color), 0 0 8px rgba(255, 255, 255, 0.8);
    }

    .custom-slider::-moz-range-track {
      width: 100%;
      height: 10px;
      border-radius: 5px;
      background: transparent;
    }

    .custom-slider::-moz-range-thumb {
      width: 22px;
      height: 22px;
      border: none;
      border-radius: 50%;
      background: var(--accent-gradient);
      box-shadow: 0 0 10px var(--accent-color);
      cursor: pointer;
      transition: transform 0.15s cubic-bezier(0.34, 1.56, 0.64, 1), box-shadow 0.15s ease;
    }

    .custom-slider::-moz-range-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 0 16px var(--accent-color), 0 0 4px rgba(255, 255, 255, 0.4);
    }

    .custom-slider::-moz-range-thumb:active {
      transform: scale(1.4);
      box-shadow: 0 0 24px var(--accent-color), 0 0 8px rgba(255, 255, 255, 0.8);
    }


  </style>
</head>
<body>
  <header>
    <div class="logo-container">
      <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" style="color: #6366f1;">
        <rect x="3" y="11" width="18" height="10" rx="2"></rect>
        <circle cx="12" cy="5" r="2"></circle>
        <path d="M12 7v4"></path>
        <line x1="8" y1="16" x2="8" y2="16"></line>
        <line x1="16" y1="16" x2="16" y2="16"></line>
      </svg>
      <span class="logo-text">ESP32 ROBOT</span>
    </div>
    <div class="header-actions">
      <a href="/wifi" class="btn-nav-header">
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
          <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
          <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
          <line x1="12" y1="20" x2="12.01" y2="20"></line>
        </svg>
        <span>配置</span>
      </a>
      <div id="status-badge" class="status-badge">
        <div class="status-dot"></div>
        <span id="status-text">未连接</span>
      </div>
    </div>
  </header>

  <main>
    <div class="dashboard-panel">
      <div class="stat-card">
        <div class="stat-icon">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <rect x="1" y="6" width="18" height="12" rx="2" ry="2"></rect>
            <line x1="23" y1="11" x2="23" y2="13"></line>
          </svg>
        </div>
        <div class="stat-content">
          <span class="stat-label">电池电量</span>
          <span id="battery-value" class="stat-value" style="color: var(--text-muted);">-- % / -- V</span>
        </div>
      </div>
      <div class="stat-card">
        <div class="stat-icon">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71"></path>
            <path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71"></path>
          </svg>
        </div>
        <div class="stat-content">
          <span class="stat-label">Agent 状态</span>
          <span id="agent-state-val" class="stat-value" style="color: var(--text-muted);">等待连接</span>
        </div>
      </div>
    </div>

    <div class="speed-control-panel" style="width: 100%; display: flex; flex-direction: column; gap: 10px; background: var(--card-bg); border: 1px solid var(--card-border); backdrop-filter: blur(12px); border-radius: 16px; padding: 16px 18px; box-shadow: 0 4px 20px rgba(0,0,0,0.15); margin-top: 8px;">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <span style="font-size: 11px; color: var(--text-muted); font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px;">最大速度倍率</span>
        <span id="speed-scale-val" style="font-family: 'Outfit', sans-serif; font-size: 14px; font-weight: 800; color: var(--accent-color);">1.0x</span>
      </div>
      <div style="display: flex; align-items: center; gap: 14px; width: 100%;">
        <span style="font-size: 11px; color: var(--text-muted); font-weight: 600;">0.2x</span>
        <input type="range" id="speed-slider" class="custom-slider" min="0.2" max="1.5" step="0.1" value="1.0" style="--value: 61.5%;">
        <span style="font-size: 11px; color: var(--text-muted); font-weight: 600;">1.5x</span>
      </div>
    </div>

    <div id="joystick-wrapper" class="joystick-container">
      <div id="joystick-zone" class="joystick-zone">
        <div class="joystick-axis-h"></div>
        <div class="joystick-axis-v"></div>
        <div id="stick" class="stick"></div>
      </div>
    </div>
  </main>

  <script>
    let ws;
    const statusBadge = document.getElementById('status-badge');
    const statusText = document.getElementById('status-text');
    const zone = document.getElementById('joystick-zone');
    const stick = document.getElementById('stick');
    const joystickWrapper = document.getElementById('joystick-wrapper');

    let isDragging = false;
    let centerX, centerY;
    const maxRadius = 140 - 45;

    let outX = 0;
    let outY = 0;
    let speedScale = 1.0;

    const speedSlider = document.getElementById('speed-slider');
    const speedScaleVal = document.getElementById('speed-scale-val');

    function updateSliderProgress(el) {
      const min = parseFloat(el.min) || 0.2;
      const max = parseFloat(el.max) || 1.5;
      const val = parseFloat(el.value);
      const percentage = ((val - min) / (max - min)) * 100;
      el.style.setProperty('--value', `${percentage}%`);
    }

    // Initialize progress bar colors on load
    updateSliderProgress(speedSlider);

    speedSlider.addEventListener('input', (e) => {
      speedScale = parseFloat(e.target.value);
      speedScaleVal.innerText = `${speedScale.toFixed(1)}x`;
      updateSliderProgress(e.target);
    });

    function connect() {
      ws = new WebSocket(`ws://${window.location.host}/ws`);
      
      ws.onopen = () => {
        statusBadge.classList.add('connected');
        statusText.innerText = "已连接";
      };
      
      ws.onclose = () => {
        statusBadge.classList.remove('connected');
        statusText.innerText = "未连接";
        setTimeout(connect, 1000);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.hasOwnProperty('battery_v') && data.hasOwnProperty('battery_p')) {
            const batValEl = document.getElementById('battery-value');
            batValEl.innerText = `${data.battery_p} % / ${data.battery_v.toFixed(2)} V`;
            if (data.battery_p >= 50) {
              batValEl.style.color = 'var(--success)';
            } else if (data.battery_p >= 20) {
              batValEl.style.color = '#f59e0b';
            } else {
              batValEl.style.color = 'var(--error)';
            }
          }
          if (data.hasOwnProperty('agent_state')) {
            let stateStr = "未知";
            let color = "var(--text-muted)";
            switch(data.agent_state) {
              case "WaitingAgent": stateStr = "等待 Agent"; color = "#f59e0b"; break;
              case "AgentAvailable": stateStr = "Agent 可用"; color = "#6366f1"; break;
              case "AgentConnected": stateStr = "连接成功"; color = "var(--success)"; break;
              case "AgentDisconnected": stateStr = "连接断开"; color = "var(--error)"; break;
            }
            const agentVal = document.getElementById('agent-state-val');
            if (agentVal) {
              agentVal.innerText = stateStr;
              agentVal.style.color = color;
            }
          }
        } catch (err) {
          // Ignore parse errors
        }
      };
    }

    function sendCommand() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        if (isDragging || (outX === 0 && outY === 0)) {
          ws.send(JSON.stringify({ x: outX * speedScale, y: outY * speedScale }));
        }
      }
    }

    setInterval(sendCommand, 50);

    function updateStickPosition(clientX, clientY) {
      let dx = clientX - centerX;
      let dy = clientY - centerY;
      const distance = Math.sqrt(dx * dx + dy * dy);
      
      if (distance > maxRadius) {
        const ratio = maxRadius / distance;
        dx *= ratio;
        dy *= ratio;
      }
      
      stick.style.transform = `translate(${dx}px, ${dy}px)`;
      
      outX = dx / maxRadius;
      outY = -(dy / maxRadius);
    }

    function resetStick() {
      isDragging = false;
      stick.classList.remove('active');
      stick.style.transform = 'translate(0px, 0px)';
      outX = 0;
      outY = 0;
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ x: 0, y: 0 }));
      }
    }

    zone.addEventListener('touchstart', (e) => {
      e.preventDefault();
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });

    zone.addEventListener('touchmove', (e) => {
      e.preventDefault();
      if (!isDragging) return;
      updateStickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });

    zone.addEventListener('touchend', (e) => {
      e.preventDefault();
      resetStick();
    });
    
    zone.addEventListener('touchcancel', (e) => {
      e.preventDefault();
      resetStick();
    });

    zone.addEventListener('mousedown', (e) => {
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.clientX, e.clientY);
    });

    document.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      updateStickPosition(e.clientX, e.clientY);
    });

    document.addEventListener('mouseup', () => {
      if (isDragging) resetStick();
    });

    connect();
  </script>
</body>
</html>
)=====";

const char WIFI_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 机器人网络配置</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@400;600;800&family=Inter:wght@400;500;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-color: #080c14;
      --card-bg: rgba(18, 24, 38, 0.6);
      --card-border: rgba(255, 255, 255, 0.08);
      --accent-color: #6366f1;
      --accent-gradient: linear-gradient(135deg, #6366f1, #a855f7);
      --text-main: #f3f4f6;
      --text-muted: #9ca3af;
      --success: #10b981;
      --error: #ef4444;
      --input-bg: rgba(10, 15, 26, 0.8);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      background-color: var(--bg-color);
      background-image: 
        radial-gradient(at 0% 0%, rgba(99, 102, 241, 0.15) 0px, transparent 50%),
        radial-gradient(at 100% 100%, rgba(168, 85, 247, 0.15) 0px, transparent 50%);
      color: var(--text-main);
      font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 20px;
    }

    main {
      width: min(460px, 100%);
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      backdrop-filter: blur(12px);
      border-radius: 20px;
      padding: 28px;
      box-shadow: 0 12px 40px rgba(0,0,0,0.4);
    }

    h1 {
      margin: 0 0 8px;
      font-family: 'Outfit', sans-serif;
      font-weight: 800;
      font-size: 26px;
      background: var(--accent-gradient);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      letter-spacing: -0.5px;
    }

    .subtitle {
      color: var(--text-muted);
      font-size: 14px;
      margin: 0 0 24px;
    }

    #status {
      font-size: 13px;
      color: var(--text-main);
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid var(--card-border);
      padding: 10px 14px;
      border-radius: 10px;
      margin-bottom: 24px;
      display: flex;
      flex-direction: column;
      gap: 4px;
    }

    .status-row {
      display: flex;
      justify-content: space-between;
    }

    .status-label {
      color: var(--text-muted);
    }

    .history-container {
      margin-bottom: 24px;
      background: rgba(255, 255, 255, 0.02);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      overflow: hidden;
    }

    .history-header {
      padding: 12px 16px;
      font-size: 13px;
      font-weight: 600;
      color: var(--text-muted);
      border-bottom: 1px solid var(--card-border);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .history-list {
      max-height: 180px;
      overflow-y: auto;
    }

    .history-item {
      padding: 12px 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid rgba(255,255,255,0.03);
      cursor: pointer;
      transition: background-color 0.2s ease;
    }

    .history-item:last-child {
      border-bottom: none;
    }

    .history-item:hover {
      background-color: rgba(99, 102, 241, 0.08);
    }

    .history-details {
      display: flex;
      flex-direction: column;
      gap: 2px;
      flex: 1;
    }

    .history-ssid {
      font-size: 14px;
      font-weight: 600;
      color: var(--text-main);
    }

    .history-sub {
      font-size: 11px;
      color: var(--text-muted);
    }

    .history-actions {
      display: flex;
      align-items: center;
    }

    .btn-delete {
      background: none;
      border: none;
      color: var(--text-muted);
      cursor: pointer;
      padding: 6px;
      border-radius: 6px;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: all 0.2s ease;
    }

    .btn-delete:hover {
      color: var(--error);
      background: rgba(239, 68, 68, 0.1);
    }

    .history-empty {
      padding: 20px;
      text-align: center;
      color: var(--text-muted);
      font-size: 13px;
    }

    /* Switch styles */
    .switch {
      position: relative;
      display: inline-block;
      width: 44px;
      height: 24px;
    }
    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }
    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: rgba(255, 255, 255, 0.1);
      transition: .4s;
      border: 1px solid var(--card-border);
    }
    .slider:before {
      position: absolute;
      content: "";
      height: 16px;
      width: 16px;
      left: 3px;
      bottom: 3px;
      background-color: var(--text-muted);
      transition: .4s;
    }
    input:checked + .slider {
      background-color: var(--accent-color);
      border-color: var(--accent-color);
    }
    input:checked + .slider:before {
      transform: translateX(20px);
      background-color: white;
    }
    .slider.round {
      border-radius: 24px;
    }
    .slider.round:before {
      border-radius: 50%;
    }

    form {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .form-group {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    label {
      font-size: 12px;
      font-weight: 600;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }

    input {
      width: 100%;
      padding: 12px 14px;
      border: 1px solid var(--card-border);
      border-radius: 10px;
      background: var(--input-bg);
      color: #fff;
      font-size: 15px;
      transition: all 0.3s ease;
    }

    input:focus {
      outline: none;
      border-color: var(--accent-color);
      box-shadow: 0 0 0 2px rgba(99, 102, 241, 0.2);
    }

    .row {
      display: grid;
      grid-template-columns: 2fr 1fr;
      gap: 12px;
    }

    .button-group {
      display: flex;
      flex-direction: column;
      gap: 10px;
      margin-top: 10px;
    }

    button {
      width: 100%;
      padding: 14px;
      border: 0;
      border-radius: 10px;
      font-weight: 600;
      font-size: 15px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.3s ease;
    }

    button[type="submit"] {
      background: var(--accent-gradient);
      color: #fff;
      box-shadow: 0 4px 15px rgba(99, 102, 241, 0.3);
    }

    button[type="submit"]:hover {
      opacity: 0.9;
      transform: translateY(-1px);
      box-shadow: 0 6px 20px rgba(99, 102, 241, 0.4);
    }

    button.secondary {
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid var(--card-border);
      color: var(--text-main);
    }

    button.secondary:hover {
      background: rgba(255, 255, 255, 0.08);
    }

    .btn-back {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      margin-top: 16px;
      color: var(--text-muted);
      text-decoration: none;
      font-size: 14px;
      font-weight: 500;
      transition: color 0.2s ease;
    }

    .btn-back:hover {
      color: var(--text-main);
    }

    #msg {
      margin-top: 16px;
      text-align: center;
      font-size: 13px;
      min-height: 20px;
      line-height: 1.5;
    }

    .msg-success { color: var(--success); }
    .msg-error { color: var(--error); }
  </style>
</head>
<body>
  <main>
    <h1>网络与 Agent 配置</h1>
    <p class="subtitle">配置机器人的 WiFi 网络及 ROS 2 Agent 地址</p>

    <div id="status">
      <div class="status-row">
        <span class="status-label">STA 连接状态:</span>
        <span id="sta-status" style="font-weight:600;">正在获取...</span>
      </div>
      <div class="status-row">
        <span class="status-label">AP 热点 IP:</span>
        <span id="ap-ip" style="font-weight:600;">正在获取...</span>
      </div>
    </div>

    <div class="history-container">
      <div class="history-header">
        <span>历史网络记录</span>
        <span id="history-count" style="font-size:11px; font-weight:normal;">0/5</span>
      </div>
      <div id="history-list" class="history-list">
      </div>
    </div>

    <form id="form">
      <div class="form-group">
        <label for="ssid">WiFi SSID (网络名称)</label>
        <input id="ssid" name="ssid" placeholder="输入 WiFi 名称" autocomplete="off" required>
      </div>
      <div class="form-group">
        <label for="password">WiFi 密码</label>
        <input id="password" name="password" type="password" placeholder="若为开放网络可留空" autocomplete="off">
      </div>
      <div class="row">
        <div class="form-group">
          <label for="agent_ip">Agent IP</label>
          <input id="agent_ip" name="agent_ip" placeholder="192.168.x.x" required>
        </div>
        <div class="form-group">
          <label for="agent_port">端口</label>
          <input id="agent_port" name="agent_port" type="number" min="1" max="65535" placeholder="8888" required>
        </div>
      </div>
      
      <div class="form-group">
        <label>VOFA+ 调试输出</label>
        <div style="display: flex; align-items: center; gap: 10px; margin-top: 4px;">
          <label class="switch">
            <input type="checkbox" id="vofa_debug" name="vofa_debug" value="true">
            <span class="slider round"></span>
          </label>
          <span style="font-size: 13px; color: var(--text-muted);">启用 Serial1 调试串口周期发送浮点波形</span>
        </div>
      </div>

      <div class="button-group">
        <button type="submit">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
            <path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"></path>
            <polyline points="17 21 17 13 7 13 7 21"></polyline>
            <polyline points="7 3 7 8 15 8"></polyline>
          </svg>
          保存并重启机器人
        </button>
        <button class="secondary" type="button" id="clear">清除配置</button>
      </div>
    </form>
    
    <div id="msg"></div>

    <a href="/" class="btn-back">
      <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <line x1="19" y1="12" x2="5" y2="12"></line>
        <polyline points="12 19 5 12 12 5"></polyline>
      </svg>
      返回摇杆控制
    </a>
  </main>

  <script>
    const msgEl = document.getElementById('msg');
    const staStatusEl = document.getElementById('sta-status');
    const apIpEl = document.getElementById('ap-ip');
    const historyListEl = document.getElementById('history-list');
    const historyCountEl = document.getElementById('history-count');
    
    const ssidInput = document.getElementById('ssid');
    const passwordInput = document.getElementById('password');
    const agentIpInput = document.getElementById('agent_ip');
    const agentPortInput = document.getElementById('agent_port');

    const MAX_HISTORY = 5;

    async function getHistoryFromServer() {
      try {
        const res = await fetch('/api/wifi/history');
        if (res.status === 200) {
          return await res.json();
        }
      } catch (err) {
        console.error('Failed to fetch history', err);
      }
      return [];
    }

    async function renderHistory() {
      const history = await getHistoryFromServer();
      historyCountEl.innerText = `${history.length}/${MAX_HISTORY}`;
      
      if (history.length === 0) {
        historyListEl.innerHTML = '<div class="history-empty">暂无保存的配置记录</div>';
        return;
      }
      
      historyListEl.innerHTML = '';
      history.forEach((item, index) => {
        const itemEl = document.createElement('div');
        itemEl.className = 'history-item';
        itemEl.innerHTML = `
          <div class="history-details">
            <span class="history-ssid">${item.ssid}</span>
            <span class="history-sub">Agent: ${item.agent_ip}:${item.agent_port}</span>
          </div>
          <div class="history-actions">
            <button class="btn-delete" title="删除记录">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <polyline points="3 6 5 6 21 6"></polyline>
                <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                <line x1="10" y1="11" x2="10" y2="17"></line>
                <line x1="14" y1="11" x2="14" y2="17"></line>
              </svg>
            </button>
          </div>
        `;
        
        itemEl.addEventListener('click', () => {
          ssidInput.value = item.ssid;
          passwordInput.value = item.password || '';
          agentIpInput.value = item.agent_ip;
          agentPortInput.value = item.agent_port;
          
          showMsg('已成功载入该配置，点击“保存并重启”生效', 'success');
        });
        
        const deleteBtn = itemEl.querySelector('.btn-delete');
        deleteBtn.addEventListener('click', async (e) => {
          e.stopPropagation();
          showMsg('正在删除历史记录...', 'success');
          try {
            const body = new URLSearchParams();
            body.append('index', index);
            const res = await fetch('/api/wifi/history/delete', { method: 'POST', body });
            if (res.status === 200) {
              await renderHistory();
              showMsg('已删除配置记录', 'success');
            } else {
              showMsg('删除历史记录失败: ' + await res.text(), 'error');
            }
          } catch (err) {
            showMsg('网络请求失败', 'error');
          }
        });
        
        historyListEl.appendChild(itemEl);
      });
    }

    function showMsg(text, type) {
      msgEl.innerText = text;
      msgEl.className = type === 'success' ? 'msg-success' : 'msg-error';
    }

    async function loadStatus() {
      try {
        const res = await fetch('/api/wifi/status');
        const data = await res.json();
        
        if (!ssidInput.value) ssidInput.value = data.ssid || '';
        if (!agentIpInput.value) agentIpInput.value = data.agent_ip || '';
        if (!agentPortInput.value) agentPortInput.value = data.agent_port || 8888;
        document.getElementById('vofa_debug').checked = data.vofa_debug || false;
        
        staStatusEl.innerText = data.connected ? `已连接 (${data.local_ip})` : '未连接';
        staStatusEl.style.color = data.connected ? 'var(--success)' : 'var(--error)';
        apIpEl.innerText = data.ap_ip || '未启用';
      } catch (err) {
        staStatusEl.innerText = '获取失败';
        staStatusEl.style.color = 'var(--error)';
      }
    }

    document.getElementById('form').addEventListener('submit', async (e) => {
      e.preventDefault();
      showMsg('正在发送保存配置请求...', 'success');
      const body = new URLSearchParams(new FormData(e.target));
      try {
        const res = await fetch('/api/wifi', { method: 'POST', body });
        const responseText = await res.text();
        if (res.status === 200) {
          showMsg(responseText, 'success');
          await renderHistory();
        } else {
          showMsg('保存失败: ' + responseText, 'error');
        }
      } catch (err) {
        showMsg('保存请求失败，请检查设备连接状态', 'error');
      }
    });

    document.getElementById('clear').addEventListener('click', async () => {
      if (confirm('确定要清除机器人的 WiFi 配置吗？重启后将使用默认出厂配置。')) {
        showMsg('正在清除配置并重启...', 'success');
        try {
          const res = await fetch('/api/wifi/clear', { method: 'POST' });
          showMsg(await res.text(), 'success');
        } catch (err) {
          showMsg('清除请求失败', 'error');
        }
      }
    });

    renderHistory();
    loadStatus();
  </script>
</body>
</html>
)=====";

} // namespace robot
