#include <WiFi.h>
#include <HTTPClient.h>

/*
to use
call
wifi_init() in setup
and then use

wifi_log(String s) to log data
*/

// Wi-Fi credentials
const char* ssid = "MeOrYou";
const char* password = "12345678";

// IP address (may change, so kept as a variable)
String ipAddress = "192.168.210.1";

// LED pin (built-in LED on most ESP32 boards)
const int ledPin = 2;

// Initialize Wi-Fi and connect
void wifi_init() {
  pinMode(ledPin, OUTPUT);
  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password);

  // Blink LED while connecting
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }

  // Connected
  Serial.println();
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Turn on LED for 1 second after connection
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
}

// Log message via HTTP GET request
void wifi_log(String msg) {
  msg.replace(" ","%20");
if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://" + ipAddress + ":5000/log?msg=" + msg;

    http.begin(url);
    int httpResponseCode = http.GET();
    http.end();
  }
}
