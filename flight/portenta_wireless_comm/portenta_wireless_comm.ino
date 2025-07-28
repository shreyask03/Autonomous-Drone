#include <WiFi.h>
#include "statusLED.h"


const char* ssid = "Ben10";
const char* password = "kl7d8339";

const char* serverIP = "192.168.1.62";
const uint16_t serverPort = 8000;

WiFiClient client;
statusLED s;

String lastReceived = "";
static constexpr int TIMEOUT = 5000; // ms
unsigned long lastConnectionCheck = 0;

void connectToWiFi() {
  s.connectingWiFi();
  WiFi.begin(ssid, password);

  int attempts = 5;
  while (WiFi.status() != WL_CONNECTED && attempts-- > 0) {
    delay(100);
  }
}

void connectToServer() {
  s.connectingServer();
  // Try multiple times to connect
  int retries = 5;
  while (!client.connect(serverIP, serverPort) && retries-- > 0) {
    delay(100);
  }
}

void setup() {
  s.init(); // initialize led pins
  // while(!Serial) delay(500);
  connectToWiFi();
  connectToServer();
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop() {
  // Serial.println("Loop Running"); // checks if loop is not getting stuck, if working properly should print every loop
  // Reconnect WiFi if dropped
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    return;
  }

  // Reconnect to server if dropped
  if (!client.connected()) {
    // Serial.println("Client disconnected! Retrying...");
    connectToServer();
    // Serial.println("Reconnected!");
    return;
  }

  // read data from server and process commands
  static String inputBuffer = "";
  while (client.available()) {
    char c = client.read();
    if (c == '\n') { // end of message reached
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        if (inputBuffer == "REQUEST,TUNE") {
          client.print("ACK\n"); // send ACK immediately to server
          delay(10);
          client.flush();


          Serial1.println(inputBuffer); // Ask Nano for current tune
          String message = waitForSerial1Message();
          if (message.startsWith("TUNE")) {

            // client.print("ACK\n"); //send ACK to server first acknowledge successful read from nano
            // delay(10);
            // client.flush();

            client.print(message + '\n');
            client.flush(); // just to avoid any issues
            waitForAck(); // wait for ACK from server
          }
        }
        else if (inputBuffer.startsWith("TUNE")) {

          client.print("ACK\n"); // send ACK immediately to server
          delay(10);
          client.flush();

          Serial1.println(inputBuffer); // Send new tune to Nano
          String message = waitForSerial1Message();
          if (message.startsWith("TUNE")) {

            // client.print("ACK\n"); // send ACK to server
            // delay(10);
            // client.flush();

            client.print(message + '\n');
            client.flush();
            waitForAck(); // wait for ACK from server
          }
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Check for dead connection (no activity for > timeout)
  if (millis() - lastConnectionCheck > TIMEOUT) {
    if (!client.connected()) {
      connectToServer();
    }
    lastConnectionCheck = millis();
  }
}


void waitForAck(){
  s.dataTransfer();
  unsigned long startTime = millis();

  while(millis() - startTime < TIMEOUT){
    if(client.available()){
      String response = client.readStringUntil('\n');
      response.trim();
      if(response == "ACK"){
        return;
      }
    }
  }
}

String waitForSerial1Message() {
  unsigned long start = millis();
  String msg = "";
  while (millis() - start < TIMEOUT) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        msg.trim();
        return msg;
      }
      msg += c;
    }
  }
  return ""; // Timeout or bad message
}
