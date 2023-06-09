#include <WiFi.h>
  
// debug log, set to 1 to enable
#define ENABLE_DEBUG_LOG 0
  
// wifi config
const char* ssid     = "RoBorregos";
const char* password = "RoBorregos2023";
    
// rs-server config
const int serverPort = 1234;
  
// rs port config
const int baudrate = 230400;
const int rs_config = SERIAL_8N1;
  
// reading buffor config
#define BUFFER_SIZE 4096
  
// global objects
WiFiServer server;
byte buff[BUFFER_SIZE];
  
void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}
  
void setup() {
  // init rs port
  Serial.begin(baudrate, rs_config);
  
  // init wifi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    debug_log("Connecting to WiFi network...");
    delay(500);
  }

#if ENABLE_DEBUG_LOG == 1
  Serial.println("connected to WiFi");
  Serial.println("IP adddr: ");
  Serial.println(WiFi.localIP());
#endif
  delay(1000);
  
  //start server
  server = WiFiServer(serverPort);
  server.begin();
  delay(1000);
  debug_log("server started");
}
  
void loop() {
  // wait for client
  WiFiClient client = server.available();
  if (!client)
    return;
  
  debug_log("client found");
  while (client.connected()) {
    int size = 0;

    // read data from wifi client and send to serial
    while ((size = client.available())) {
      size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
      client.read(buff, size);
      Serial.write(buff, size);
      Serial.flush();
    }
    
    // read data from serial and send to wifi client
    while ((size = Serial.available())) {
      size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
      Serial.readBytes(buff, size);
      client.write(buff, size);
      client.flush();
    }
  }
  debug_log("client disconnected");
  client.stop();
}
