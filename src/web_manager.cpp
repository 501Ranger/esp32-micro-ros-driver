#include "web_manager.h"
#include "robot_config.h"
#include <WiFi.h>

namespace robot {

using namespace robot_config;

const char* INDEX_HTML PROGMEM = R"=====(
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

    .estop-section {
      width: 100%;
      display: flex;
      justify-content: center;
      align-items: center;
    }

    .estop-outer {
      width: 110px;
      height: 110px;
      border-radius: 50%;
      background: radial-gradient(circle, #fcd34d 0%, #d97706 100%);
      border: 5px solid #111;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 
        0 8px 16px rgba(0,0,0,0.4),
        inset 0 2px 5px rgba(255,255,255,0.4);
      cursor: pointer;
      position: relative;
      transition: transform 0.1s ease;
    }

    .estop-outer:active {
      transform: scale(0.97);
    }

    .estop-button {
      width: 76px;
      height: 76px;
      border-radius: 50%;
      background: radial-gradient(circle at 35% 35%, #ef4444 0%, #991b1b 100%);
      border: 3px solid #7f1d1d;
      box-shadow: 
        inset 0 3px 5px rgba(255,255,255,0.3),
        0 4px 6px rgba(0,0,0,0.5);
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      color: #fff;
      font-family: 'Outfit', sans-serif;
      font-weight: 800;
      font-size: 13px;
      letter-spacing: 0.5px;
      text-shadow: 0 1px 3px rgba(0,0,0,0.6);
      transition: all 0.2s ease;
    }

    .estop-text-top {
      font-size: 10px;
      opacity: 0.8;
      margin-bottom: -2px;
    }

    .estop-outer.active {
      background: radial-gradient(circle, #fef08a 0%, #ca8a04 100%);
      animation: estop-glow 1s infinite alternate;
    }

    .estop-outer.active .estop-button {
      background: radial-gradient(circle at 35% 35%, #dc2626 0%, #b91c1c 100%);
      transform: translateZ(0) scale(0.92);
      box-shadow: 
        inset 0 5px 8px rgba(0,0,0,0.8),
        0 2px 3px rgba(0,0,0,0.3);
      border-color: #991b1b;
    }

    @keyframes estop-glow {
      0% { box-shadow: 0 0 15px rgba(239, 68, 68, 0.4), 0 8px 16px rgba(0,0,0,0.4); }
      100% { box-shadow: 0 0 30px rgba(239, 68, 68, 0.8), 0 8px 16px rgba(0,0,0,0.4); }
    }

    .estop-label-text {
      position: absolute;
      bottom: -22px;
      font-size: 10px;
      font-weight: 600;
      color: var(--text-muted);
      letter-spacing: 1px;
      text-transform: uppercase;
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
      width: 220px;
      height: 220px;
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
      width: 76px;
      height: 76px;
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

    .joystick-disabled .joystick-zone {
      border-color: var(--error);
      opacity: 0.5;
    }
    .joystick-disabled .stick {
      background: #1f2937;
      border-color: #374151;
      box-shadow: none;
    }
    .joystick-disabled .stick::after {
      background: #4b5563;
      box-shadow: none;
      opacity: 0.3;
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
            <polyline points="22 12 18 12 15 21 9 3 6 12 2 12"></polyline>
          </svg>
        </div>
        <div class="stat-content">
          <span class="stat-label">控制状态</span>
          <span id="control-state-val" class="stat-value">就绪</span>
        </div>
      </div>
    </div>

    <div class="estop-section">
      <div id="estop-btn" class="estop-outer">
        <div class="estop-button">
          <span class="estop-text-top">EMERGENCY</span>
          <span>STOP</span>
        </div>
        <span class="estop-label-text">急停按钮</span>
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
    const ctrlStateVal = document.getElementById('control-state-val');
    const zone = document.getElementById('joystick-zone');
    const stick = document.getElementById('stick');
    const estopBtn = document.getElementById('estop-btn');
    const joystickWrapper = document.getElementById('joystick-wrapper');

    let isDragging = false;
    let centerX, centerY;
    const maxRadius = 110 - 38;

    let outX = 0;
    let outY = 0;
    let isEstop = false;

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
    }

    function sendCommand() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        if (isEstop) {
          ws.send(JSON.stringify({ x: 0, y: 0 }));
        } else if (isDragging || (outX === 0 && outY === 0)) {
          ws.send(JSON.stringify({ x: outX, y: outY }));
        }
      }
    }

    setInterval(sendCommand, 50);

    function updateStickPosition(clientX, clientY) {
      if (isEstop) return;
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

    estopBtn.addEventListener('click', () => {
      isEstop = !isEstop;
      if (isEstop) {
        estopBtn.classList.add('active');
        joystickWrapper.classList.add('joystick-disabled');
        ctrlStateVal.innerText = "急停触发";
        ctrlStateVal.style.color = "var(--error)";
        resetStick();
        if (navigator.vibrate) navigator.vibrate([100, 50, 100]);
      } else {
        estopBtn.classList.remove('active');
        joystickWrapper.classList.remove('joystick-disabled');
        ctrlStateVal.innerText = "就绪";
        ctrlStateVal.style.color = "var(--text-main)";
        if (navigator.vibrate) navigator.vibrate(50);
      }
    });

    zone.addEventListener('touchstart', (e) => {
      if (isEstop) return;
      e.preventDefault();
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });

    zone.addEventListener('touchmove', (e) => {
      if (isEstop) return;
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
      if (isEstop) return;
      const rect = zone.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
      isDragging = true;
      stick.classList.add('active');
      updateStickPosition(e.clientX, e.clientY);
    });

    document.addEventListener('mousemove', (e) => {
      if (!isDragging || isEstop) return;
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

const char* WIFI_HTML PROGMEM = R"=====(
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

WebManager::WebManager() : server_(80), ws_("/ws") {}

WebManager::~WebManager() {}

void WebManager::begin(WifiConfigManager &wifi_config_manager) {
  wifi_config_manager_ = &wifi_config_manager;
  pinMode(GAMEPAD_LED_PIN, OUTPUT);
  digitalWrite(GAMEPAD_LED_PIN, HIGH);
  ledcSetup(BUZZER_PWM_CHANNEL, 1000, 10);
  ledcAttachPin(GAMEPAD_BUZZER_PIN, BUZZER_PWM_CHANNEL);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);

  ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    this->onWebSocketEvent(server, client, type, arg, data, len);
  });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server_.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", WIFI_HTML);
  });
  server_.on("/api/wifi/status", HTTP_GET, [this](AsyncWebServerRequest *request){
    this->sendWifiStatus(request);
  });
  server_.on("/api/wifi/clear", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiClear(request);
  });
  server_.on("/api/wifi/history", HTTP_GET, [this](AsyncWebServerRequest *request){
    this->handleWifiHistoryGet(request);
  });
  server_.on("/api/wifi/history/delete", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiHistoryDelete(request);
  });
  server_.on("/api/wifi", HTTP_POST, [this](AsyncWebServerRequest *request){
    this->handleWifiSave(request);
  });

  server_.begin();
#ifndef USE_SERIAL_TRANSPORT
  Serial.println("Web Server Started on port 80");
#endif
}

void WebManager::loop() {
  ws_.cleanupClients();
  if (restart_at_ms_ != 0 && millis() >= restart_at_ms_) {
    ESP.restart();
  }

  // Dynamic MDNS & NetBIOS (NBNS) setup when IP is ready
  static bool names_started = false;
  bool has_ip = (WiFi.status() == WL_CONNECTED) || 
                ((WiFi.getMode() & WIFI_AP) && WiFi.softAPIP() != IPAddress(0, 0, 0, 0));
  
  if (has_ip) {
    if (!names_started) {
      // 1. Initialize/Restart MDNS
      if (MDNS.begin("esp32robot")) {
        MDNS.addService("http", "tcp", 80);
#ifndef USE_SERIAL_TRANSPORT
        Serial.print("mDNS responder started: http://esp32robot.local/ (IP: ");
        Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
        Serial.println(")");
#endif
      }
      
      // 2. Initialize NetBIOS (NBNS) - fallback for Windows
      NBNS.begin("esp32robot");
#ifndef USE_SERIAL_TRANSPORT
      Serial.println("NetBIOS responder started: http://esp32robot/");
#endif
      
      names_started = true;
    }
  } else {
    if (names_started) {
      MDNS.end();
      NBNS.end();
      names_started = false;
    }
  }
  
  // Timeout protection: if no commands received for a while, stop the robot
  if (is_active_ && millis() - last_command_ms_ > COMMAND_TIMEOUT_MS) {
    linear_out_ = 0.0f;
    angular_out_ = 0.0f;
    is_active_ = false;
#ifndef USE_SERIAL_TRANSPORT
    Serial.println("Web Joystick Timeout - Stopping");
#endif
  }
  
  updateFeedback();
}

bool WebManager::getVelocity(float &linear_mps, float &angular_radps) const {
  linear_mps = linear_out_;
  angular_radps = angular_out_;
  return is_active_;
}

void WebManager::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                                  AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
#ifndef USE_SERIAL_TRANSPORT
    Serial.printf("WS Client Connected: %u\n", client->id());
#endif
    connected_clients_++;
    if (connected_clients_ == 1) {
      playConnectSound();
    }
  } else if (type == WS_EVT_DISCONNECT) {
#ifndef USE_SERIAL_TRANSPORT
    Serial.printf("WS Client Disconnected: %u\n", client->id());
#endif
    if (connected_clients_ > 0) connected_clients_--;
    if (connected_clients_ == 0) {
      linear_out_ = 0.0f;
      angular_out_ = 0.0f;
      is_active_ = false;
    }
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (!error) {
      if (doc.containsKey("x") && doc.containsKey("y")) {
        float x = doc["x"];
        float y = doc["y"];
        
        Serial.printf("Web Joystick: x=%.2f, y=%.2f\n", x, y);
        
        linear_out_ = y * 1.0f;
        angular_out_ = -x * 1.5f;
        
        // Update state
        is_active_ = (fabs(linear_out_) > 0.01f || fabs(angular_out_) > 0.01f);
        last_command_ms_ = millis();
      }
    } else {
#ifndef USE_SERIAL_TRANSPORT
      Serial.print("JSON Parse Error: ");
      Serial.println(error.c_str());
#endif
    }
  }
}

void WebManager::handleWifiSave(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }
  if (!request->hasParam("ssid", true) ||
      !request->hasParam("agent_ip", true) ||
      !request->hasParam("agent_port", true)) {
    request->send(400, "text/plain", "Missing ssid, agent_ip, or agent_port");
    return;
  }

  NetworkConfig config;
  config.ssid = request->getParam("ssid", true)->value();
  config.ssid.trim();
  config.password = request->hasParam("password", true) ?
      request->getParam("password", true)->value() : "";
  config.agent_ip = request->getParam("agent_ip", true)->value();
  config.agent_ip.trim();
  long requested_port = request->getParam("agent_port", true)->value().toInt();
  if (requested_port < 1) {
    requested_port = 1;
  } else if (requested_port > 65535) {
    requested_port = 65535;
  }
  config.agent_port = static_cast<uint16_t>(requested_port);
  config.has_wifi = config.ssid.length() > 0;

  IPAddress parsed_agent_ip;
  if (!config.has_wifi || !parsed_agent_ip.fromString(config.agent_ip)) {
    request->send(400, "text/plain", "Invalid WiFi SSID or Agent IP");
    return;
  }

  if (!wifi_config_manager_->save(config)) {
    request->send(500, "text/plain", "Failed to save WiFi config");
    return;
  }

  wifi_config_manager_->addToHistory(config);

  restart_at_ms_ = millis() + 1200;
  request->send(200, "text/plain", "Saved. The robot will restart and connect with the new settings.");
}

void WebManager::handleWifiClear(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  if (!wifi_config_manager_->clear()) {
    request->send(500, "text/plain", "Failed to clear WiFi config");
    return;
  }

  restart_at_ms_ = millis() + 1200;
  request->send(200, "text/plain", "Cleared. The robot will restart and use the built-in fallback settings.");
}

void WebManager::handleWifiHistoryGet(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  std::vector<NetworkConfig> history = wifi_config_manager_->getHistory();

  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto &item : history) {
    JsonObject obj = arr.createNestedObject();
    obj["ssid"] = item.ssid;
    obj["password"] = item.password;
    obj["agent_ip"] = item.agent_ip;
    obj["agent_port"] = item.agent_port;
  }

  String output;
  serializeJson(doc, output);
  request->send(200, "application/json", output);
}

void WebManager::handleWifiHistoryDelete(AsyncWebServerRequest *request) {
  if (wifi_config_manager_ == nullptr) {
    request->send(500, "text/plain", "WiFi config manager is not ready");
    return;
  }

  if (!request->hasParam("index", true)) {
    request->send(400, "text/plain", "Missing index parameter");
    return;
  }

  int index = request->getParam("index", true)->value().toInt();
  if (index < 0) {
    request->send(400, "text/plain", "Invalid index");
    return;
  }

  if (!wifi_config_manager_->deleteFromHistory(static_cast<size_t>(index))) {
    request->send(500, "text/plain", "Failed to delete history item");
    return;
  }

  request->send(200, "text/plain", "Deleted successfully");
}

void WebManager::sendWifiStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<384> doc;
  const NetworkConfig config = wifi_config_manager_ != nullptr ? wifi_config_manager_->load() : NetworkConfig();

  doc["ssid"] = config.ssid;
  doc["agent_ip"] = config.agent_ip;
  doc["agent_port"] = config.agent_port;
  doc["from_flash"] = config.loaded_from_flash;
  doc["connected"] = WiFi.status() == WL_CONNECTED;
  doc["local_ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  doc["ap_ip"] = WiFi.softAPIP().toString();

  String output;
  serializeJson(doc, output);
  request->send(200, "application/json", output);
}

void WebManager::updateFeedback() {
  if (connected_clients_ == 0) {
    digitalWrite(GAMEPAD_LED_PIN, HIGH); // Off
  } else {
    digitalWrite(GAMEPAD_LED_PIN, LOW); // On
  }

  if (current_tone_idx_ != -1) {
    if (millis() - tone_start_ms_ > connect_sequence_[current_tone_idx_].duration_ms) {
      current_tone_idx_++;
      if (current_tone_idx_ >= 3) {
        current_tone_idx_ = -1;
        ledcWrite(BUZZER_PWM_CHANNEL, 0);
      } else {
        ledcWriteTone(BUZZER_PWM_CHANNEL, connect_sequence_[current_tone_idx_].frequency);
        ledcWrite(BUZZER_PWM_CHANNEL, 512); // 50% duty cycle
        tone_start_ms_ = millis();
      }
    }
  }
}

void WebManager::playConnectSound() {
  current_tone_idx_ = 0;
  tone_start_ms_ = millis();
  ledcWriteTone(BUZZER_PWM_CHANNEL, connect_sequence_[0].frequency);
  ledcWrite(BUZZER_PWM_CHANNEL, 512);
}

}  // namespace robot
