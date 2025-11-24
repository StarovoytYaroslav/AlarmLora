#pragma once

#include <ESPAsyncWebServer.h>

// --- HTML Page (Stored in Flash) ---
// We use R"raw(...)raw" to write multi-line HTML/JS easily
const char index_html[] PROGMEM = R"raw(
<!DOCTYPE HTML><html>
<head>
  <title>LoRa Map Master</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; }
    .card { background: #f4f4f4; max-width: 400px; margin: 0 auto; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2); }
    h1 { color: #003366; }
    p { font-size: 1.2rem; }
    .val { color: #d63031; font-weight: bold; }
  </style>
</head>
<body>
  <div class="card">
    <h1>LoRa Master Node</h1>
    <p>Node ID: <span id="id" class="val">Waiting...</span></p>
    <p>Message: <span id="msg" class="val">-</span></p>
    <p>RSSI: <span id="rssi" class="val">-</span> dBm</p>
    <p>Status: <span id="status" style="font-size:0.8rem; color:grey">Disconnected</span></p>
  </div>

<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;

  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
  }

  function onOpen(event) {
    document.getElementById('status').innerHTML = "Connected via WebSocket";
    document.getElementById('status').style.color = "green";
  }

  function onClose(event) {
    document.getElementById('status').innerHTML = "Connection Lost. Retrying...";
    document.getElementById('status').style.color = "red";
    setTimeout(initWebSocket, 2000);
  }

  function onMessage(event) {
    // We expect data in format: "ID,MESSAGE,RSSI"
    var data = event.data.split(","); 
    document.getElementById('id').innerHTML = data[0];
    document.getElementById('msg').innerHTML = data[1];
    document.getElementById('rssi').innerHTML = data[2];
    
    // Blink visual effect
    document.querySelector('.card').style.backgroundColor = "#e8f5e9";
    setTimeout(() => { document.querySelector('.card').style.backgroundColor = "#f4f4f4"; }, 200);
  }

  window.addEventListener('load', onLoad);
  function onLoad(event) { initWebSocket(); }
</script>
</body>
</html>
)raw";