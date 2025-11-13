#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Replace with your network credentials
const char* ssid = "Dart_Yara";
const char* password = "2358132134Me";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// NEW: HTML for the map page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  
  <style>
    /* 3. Make the map fill the entire page */
    html, body {
      height: 100%;
      margin: 0;
    }
    #map {
      height: 100%;
      width: 100%;
    }
  </style>
</head>
<body>
  
  <div id="map"></div>

  <script>
    // Initialize the map and set its view (latitude, longitude, zoom level)
    // This example is centered on London (51.505, -0.09) at zoom 13
    var map = L.map('map').setView([51.505, -0.09], 13);

    // Add a "tile layer" from OpenStreetMap
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);

    // You can add a simple marker
    var marker = L.marker([51.505, -0.09]).addTo(map);
    marker.bindPopup("<b>Hello!</b><br>I am a popup.").openPopup();
    
  </script>
</body>
</html>
)rawliteral";


void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    // We use send_P because the HTML is stored in PROGMEM (program memory)
    request->send_P(200, "text/html", index_html);
  });

  // Start the server
  server.begin();
}

void loop() {
  // Loop is empty, as the AsyncWebServer runs in the background
}