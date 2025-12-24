/**
 * Electric Parking Brake Telemetry Server
 * 
 * Simple Node.js server to collect and track brake usage data.
 * Tracks global counts across all devices.
 * 
 * Run: npm install express
 *      node server.js
 * 
 * Default port: 3000
 */

const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
app.use(express.json());

const PORT = process.env.PORT || 3000;
const DATA_FILE = path.join(__dirname, 'brake_data.json');

// Initialize or load data
let data = {
  globalStats: {
    totalApplyCount: 0,
    totalReleaseCount: 0,
    totalErrors: 0,
    totalDevices: 0,
    serverStartTime: new Date().toISOString(),
    lastEventTime: null
  },
  devices: {},
  recentEvents: []
};

// Load existing data
function loadData() {
  try {
    if (fs.existsSync(DATA_FILE)) {
      const fileData = fs.readFileSync(DATA_FILE, 'utf8');
      data = JSON.parse(fileData);
      console.log('Loaded existing data:', data.globalStats);
    }
  } catch (err) {
    console.log('Starting with fresh data');
  }
}

// Save data to file
function saveData() {
  try {
    fs.writeFileSync(DATA_FILE, JSON.stringify(data, null, 2));
  } catch (err) {
    console.error('Error saving data:', err);
  }
}

loadData();

// API endpoint for telemetry
app.post('/api/brake-telemetry', (req, res) => {
  const telemetry = req.body;
  const deviceId = telemetry.device_id;
  const eventType = telemetry.event;
  
  console.log(`\n[${new Date().toISOString()}] Event from ${deviceId}: ${eventType}`);
  
  // Initialize device if new
  if (!data.devices[deviceId]) {
    data.devices[deviceId] = {
      firstSeen: new Date().toISOString(),
      lastSeen: null,
      firmware: telemetry.firmware,
      applyCount: 0,
      releaseCount: 0,
      errorCount: 0,
      lastEvent: null,
      lastVoltage: null,
      lastTemperature: null
    };
    data.globalStats.totalDevices++;
    console.log(`  New device registered! Total devices: ${data.globalStats.totalDevices}`);
  }
  
  const device = data.devices[deviceId];
  device.lastSeen = new Date().toISOString();
  device.firmware = telemetry.firmware;
  device.lastEvent = eventType;
  
  // Update sensor readings
  if (telemetry.sensors) {
    device.lastVoltage = telemetry.sensors.battery_voltage;
    device.lastTemperature = telemetry.sensors.temperature;
    device.lastCurrent = telemetry.sensors.current_amps;
  }
  
  // Update WiFi info
  if (telemetry.wifi) {
    device.lastRSSI = telemetry.wifi.rssi;
    device.lastIP = telemetry.wifi.ip;
  }
  
  // Update device stats from payload
  if (telemetry.stats) {
    device.applyCount = telemetry.stats.apply_count;
    device.releaseCount = telemetry.stats.release_count;
    device.errorCount = telemetry.stats.error_count;
    device.totalRuntimeMs = telemetry.stats.total_runtime_ms;
  }
  
  // Track global counts by event type
  if (eventType === 'apply') {
    data.globalStats.totalApplyCount++;
    console.log(`  Global apply count: ${data.globalStats.totalApplyCount}`);
  } else if (eventType === 'release') {
    data.globalStats.totalReleaseCount++;
    console.log(`  Global release count: ${data.globalStats.totalReleaseCount}`);
  } else if (eventType === 'error') {
    data.globalStats.totalErrors++;
  }
  
  data.globalStats.lastEventTime = new Date().toISOString();
  
  // Store recent events (keep last 100)
  data.recentEvents.unshift({
    time: new Date().toISOString(),
    deviceId: deviceId,
    event: eventType,
    voltage: telemetry.sensors?.battery_voltage,
    brakeApplied: telemetry.brake_applied
  });
  if (data.recentEvents.length > 100) {
    data.recentEvents = data.recentEvents.slice(0, 100);
  }
  
  // Save to disk
  saveData();
  
  // Respond with global stats (device can display these)
  res.json({
    success: true,
    global_apply_count: data.globalStats.totalApplyCount,
    global_release_count: data.globalStats.totalReleaseCount,
    total_devices: data.globalStats.totalDevices,
    message: 'Telemetry received'
  });
});

// Dashboard endpoint - view all stats
app.get('/api/stats', (req, res) => {
  res.json(data);
});

// Simple HTML dashboard
app.get('/', (req, res) => {
  const html = `
<!DOCTYPE html>
<html>
<head>
  <title>Electric Parking Brake Telemetry</title>
  <meta http-equiv="refresh" content="10">
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; 
           max-width: 1200px; margin: 0 auto; padding: 20px; background: #1a1a2e; color: #eee; }
    h1 { color: #00d4ff; text-align: center; }
    .stats-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin: 20px 0; }
    .stat-card { background: #16213e; padding: 20px; border-radius: 10px; text-align: center; border: 1px solid #0f3460; }
    .stat-value { font-size: 3em; font-weight: bold; color: #00d4ff; }
    .stat-label { color: #888; margin-top: 5px; }
    .devices { margin-top: 30px; }
    .device { background: #16213e; padding: 15px; margin: 10px 0; border-radius: 8px; border-left: 4px solid #00d4ff; }
    .device-id { font-weight: bold; color: #00d4ff; }
    .device-stats { display: flex; gap: 20px; margin-top: 10px; flex-wrap: wrap; }
    .device-stat { background: #0f3460; padding: 5px 10px; border-radius: 5px; }
    .events { margin-top: 30px; }
    .event { padding: 8px; border-bottom: 1px solid #333; font-family: monospace; font-size: 0.9em; }
    .event:nth-child(odd) { background: #16213e; }
    .event-apply { color: #4ade80; }
    .event-release { color: #fbbf24; }
    .event-error { color: #f87171; }
    .event-boot { color: #60a5fa; }
    .event-heartbeat { color: #888; }
    .online { color: #4ade80; }
    .offline { color: #f87171; }
  </style>
</head>
<body>
  <h1>🚗 Electric Parking Brake Telemetry</h1>
  
  <div class="stats-grid">
    <div class="stat-card">
      <div class="stat-value">${data.globalStats.totalApplyCount}</div>
      <div class="stat-label">Total Applies</div>
    </div>
    <div class="stat-card">
      <div class="stat-value">${data.globalStats.totalReleaseCount}</div>
      <div class="stat-label">Total Releases</div>
    </div>
    <div class="stat-card">
      <div class="stat-value">${data.globalStats.totalDevices}</div>
      <div class="stat-label">Devices</div>
    </div>
    <div class="stat-card">
      <div class="stat-value">${data.globalStats.totalErrors}</div>
      <div class="stat-label">Errors</div>
    </div>
  </div>
  
  <div class="devices">
    <h2>📱 Registered Devices</h2>
    ${Object.entries(data.devices).map(([id, dev]) => {
      const lastSeenDate = new Date(dev.lastSeen);
      const isOnline = (Date.now() - lastSeenDate.getTime()) < 120000; // 2 minutes
      return `
      <div class="device">
        <div class="device-id">${id} <span class="${isOnline ? 'online' : 'offline'}">${isOnline ? '● Online' : '○ Offline'}</span></div>
        <div style="color:#888; font-size:0.9em;">Firmware: ${dev.firmware} | Last seen: ${lastSeenDate.toLocaleString()}</div>
        <div class="device-stats">
          <div class="device-stat">Applies: ${dev.applyCount}</div>
          <div class="device-stat">Releases: ${dev.releaseCount}</div>
          <div class="device-stat">Errors: ${dev.errorCount}</div>
          <div class="device-stat">Voltage: ${dev.lastVoltage?.toFixed(1) || '--'}V</div>
          <div class="device-stat">RSSI: ${dev.lastRSSI || '--'} dBm</div>
        </div>
      </div>`;
    }).join('')}
  </div>
  
  <div class="events">
    <h2>📊 Recent Events</h2>
    ${data.recentEvents.slice(0, 20).map(e => `
      <div class="event">
        <span style="color:#666">${new Date(e.time).toLocaleString()}</span>
        <span style="color:#00d4ff">${e.deviceId}</span>
        <span class="event-${e.event}">${e.event.toUpperCase()}</span>
        ${e.voltage ? `<span style="color:#888">${e.voltage.toFixed(1)}V</span>` : ''}
      </div>
    `).join('')}
  </div>
  
  <p style="text-align:center; color:#666; margin-top:30px;">
    Server started: ${data.globalStats.serverStartTime}<br>
    Last event: ${data.globalStats.lastEventTime || 'None yet'}<br>
    Page auto-refreshes every 10 seconds
  </p>
</body>
</html>`;
  res.send(html);
});

// Start server
app.listen(PORT, () => {
  console.log('');
  console.log('╔══════════════════════════════════════════════════╗');
  console.log('║   Electric Parking Brake Telemetry Server        ║');
  console.log('╚══════════════════════════════════════════════════╝');
  console.log(`Server running on http://localhost:${PORT}`);
  console.log(`Dashboard: http://localhost:${PORT}/`);
  console.log(`API endpoint: http://localhost:${PORT}/api/brake-telemetry`);
  console.log('');
  console.log('Global Stats:', data.globalStats);
  console.log('');
  console.log('Waiting for telemetry...');
});
