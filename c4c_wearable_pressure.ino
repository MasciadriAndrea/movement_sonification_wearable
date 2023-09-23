/*
 *   C4C - TSoftware wearable devices
 *   V: 1.0
 *   Authors: Andrea Masciadri, Federico Bianchi 
 *   Contacts: <name>.<surname>@polimi.it
 *   Da eseguire con l'applicazione mobile C4C: Contact emanuele.debernardi@polimi.it 
 *  
 */
#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define LED 13
#define LED_RED 12
#define VERBOSE_DATA false
#define VERBOSE_LOG true
#define MAX_DATASET_LEN 10000
char ssid[] = "c4cHotspot";      // Wifi SSID
char pass[] = "c4cPassword";       // Wifi password
int status = WL_IDLE_STATUS;
short dataset[MAX_DATASET_LEN];
short dataPosition = 0;
byte *iparr;
// Initialize the Wifi client
unsigned int localPort = 20001;      // local port to listen for UDP packets
unsigned int serverPort = 20001;      // server port to write UDP packets
IPAddress server(192, 168, 1, 69); // IP server
const int PACKET_SIZE = 1; // 1 bytes of the message
byte packetBuffer[PACKET_SIZE]; //buffer to hold incoming and outgoing packets
char inPacketBuffer[1]; //buffer to hold incoming packet
int state = 0;
int peak = 0;
WiFiUDP Udp;
long nt, t, time_connected, startExe, previous_peak = 0;
int ts = 10; // questo parametro è trovato dai dati, non lo si può cambiare
int fsrAnalogPin = 0; // FSR is connected to analog 0
int fsrReading;      // the analog reading from the FSR resistor divider
int appoggiato = 0;
char foot= '1'; //0 destra 1 sinistra

bool getmyip(){
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    return true;
  }
  return false;
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  Serial.begin(9600);
  digitalWrite(LED, LOW);
  digitalWrite(LED_RED, LOW);
  delay(1000);

  //-----------------------  IMU
  // check for the IMU module:
  if (!IMU.begin()) {
    printLog("Failed to initialize IMU!"); 
    // don't continue
    while (true);
  }


  //-----------------------  WIFI
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    printLog("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    //Serial.println("Please upgrade the firmware");
  }


  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    printLog(">> WIFI connection to to SSID: ");
    printLog(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(5000);
  }
  printLog("[info] WIFI connected");
  digitalWrite(LED, HIGH);
  
  Udp.begin(localPort);
  
  while(not getmyip()){
    printLog("<< waiting for SERVER BROADCAST MESSAGE");
    delay(1000);
  }
  server = Udp.remoteIP();
  printLog("[info] ----- Ip server:");
  //printLog(server);
  printLog(">> Sending pairing message to server");
  sendpacket(false);
  printLog("[info] ----- IDLE STATE -----");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED){
    digitalWrite(LED, LOW);
    printLog("[info] Lost Wifi Connection");
      while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, pass);
        delay(10000);
      }
    digitalWrite(LED, HIGH);
    printLog("[info] WIFI connected");
  }
  if(state==0){
    // IDLE STATE:
    // Waiting for start message from server
    char command = receiveMessageFromServer();
    digitalWrite(LED_RED, LOW);
    if(command=='a'){
      // No command received
      delay(1000);
      printLog(">> Sending pairing message to server");
      sendpacket(false);
    } else {
      if(inPacketBuffer[0]=='1'){
        digitalWrite(LED_RED, HIGH);
        printLog("<< Received command from server - Start Exercise");
        printLog("[info] ----- RECORDING STATE -----");
        dataPosition = 0;
        previous_peak = 0;
        state = 1;
        startExe = millis();
        Serial.print("START RECORD: ");
        Serial.println(startExe);
      }else{
        printLog("<< Received command from server - Uknown command: ");
        printLog(inPacketBuffer);
      }
    }
    
  }
  if(state==1){
    // RECORDING STATE:
    // Recording IMU DATA, peak detection and Udp transmission
    float x, y, z;
    short datas;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z); //La lettura è in g, quindi non mi aspetto valori superiori a +-6
      long now = millis();
      nt = now - startExe;
      long campionamento = nt-t;
      t = nt;
      double data = sqrt(((x) * (x)) + ((y) * (y)) + ((z) * (z)));
      if(dataPosition < MAX_DATASET_LEN){
        datas = short(data*1000);
        dataset[dataPosition] = datas;
        dataPosition += 1;
      }

      int peak_m = 0;
      fsrReading = analogRead(fsrAnalogPin);
      Serial.println(fsrReading);
      if((fsrReading<1)or(fsrReading>1022)){
        //wait
      }else{
        if(fsrReading>800){  //>300 original .... <2 others now 850
          //touching the ground
          if(appoggiato == 1){
            //the foot was already touching the ground
          }else{
            //footstep
            if(nt-previous_peak>100){
              appoggiato = 1;
              peak_m = 1;
              previous_peak = nt;
              printLog("Appoggio");
              sendpacket_step(1, dataPosition);
            }else{
              //Serial.print("NOW: ");
              //Serial.println(now);
              //Serial.print("NT: ");
              //Serial.println(nt);
              //Serial.print("previous_peak: ");
              //Serial.println(previous_peak);
              //Serial.print("diff: (>50)");
              //Serial.println(nt-previous_peak);
            }
          }
        }
        if(fsrReading<700){  //<50 original .....<2 o >300 0thers now 700
          //the foot is raised
          if(appoggiato==1){
            appoggiato=0;
            printLog("Alzo");
            sendpacket_step(2, dataPosition);
          }else{
            // the foot was already raised
          }
        }
      }
      if(VERBOSE_DATA){
        Serial.print(z); // print data
        Serial.print(",");
        Serial.print(data); // print data
        Serial.print(",");
        Serial.println(peak_m); // print peak status
      }
      delay(1);
    }
    // exit state for state 1
    char command = receiveMessageFromServer();
    if(command=='3'){
      printLog("<< Received command from server - Stop recording, No data transmission");
      printLog("[info] ----- IDLE STATE -----");
      printLog("[info] stored data:");
      printLog(String(dataPosition));
      state = 0;
    }
    if(command=='2'){
      printLog("<< Received command from server - Stop recording, Transmit data");
      printLog("[info] ----- SYNCH STATE -----");
      printLog("[info] stored data:");
      printLog(String(dataPosition));
      state = 2;
    }
    if(command=='9'){
      printLog("<< Received command from server - Reset");
      printLog("[info] ----- RESET -----");
      state = 0;
    }
  
  }
  if(state==2){
    // WAIT STATE: waiting for data transmission:
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    delay(500);
    char command = receiveMessageFromServer();
    printLog("[info] ----- WAITING STATE -----");
    printLog("[info] waiting for data transmission:");
    if(command=='4'){
      printLog("<< Received command from server - It's your turn");
      printLog(String(dataPosition));
      state = 3;
    }
    if(command=='9'){
      printLog("<< Received command from server - Reset");
      printLog("[info] ----- RESET -----");
      state = 0;
    }
  }
  if(state==3){
    // SYNCH STATE:
    // Sending recorded Data to Server
    printLog(">> Sending stored data to server");
    send_all_data();
    state = 0;
  }
  
}


void printLog(String msg){
  if(VERBOSE_LOG){
    Serial.println(msg);
  }
}


char receiveMessageFromServer(){
  int packetSize = Udp.parsePacket();
    if (packetSize) {
      // read the packet into packetBufffer
      int len = Udp.read(inPacketBuffer, 1);
      if (len > 0) {
        inPacketBuffer[len] = 0;
      }
      return inPacketBuffer[0];
    }
   return 'a';
}


unsigned long sendpacket(bool peak) {
  packetBuffer[0] = peak;
  Udp.beginPacket(server, serverPort);
  Udp.write(packetBuffer, PACKET_SIZE);
  Udp.endPacket();
}


unsigned long sendpacket_step(short command, short dp) {
  char package[5];
  memcpy(&package[0],(char *)&command,2);
  memcpy(&package[2],(char *)&dp,2);
  memcpy(&package[4],(char *)&foot,1);
  Udp.beginPacket(server, serverPort);
  Udp.write((char *)package, sizeof(package));
  Udp.endPacket();
}


unsigned long send_all_data() {
  char package[9];
  int i=0;
  int c=0;
  while(i<dataPosition){
    if(c%10==0) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
    }
    char sensorData[] = {};
    Udp.beginPacket(server, serverPort);
    memcpy(&package[0],(char *)&foot,1);
    memcpy(&package[1],(char *)&i,2);
    memcpy(&package[3],(char *)&dataset[i],2);
    i = i+1;
    if((i)<dataPosition){
      memcpy(&package[5],(char *)&dataset[i],2);
    }else{
      memcpy(&package[5],(char *)"00",2);
    }
    i = i+1;
    if((i)<dataPosition){
      memcpy(&package[7],(char *)&dataset[i],2);
    }else{
      memcpy(&package[7],(char *)"00",2);
    }
    Udp.write((char *)package, sizeof(package));
    Udp.endPacket();
    i = i+1;
    c = c+1;
    delay(2);
  }
  printLog("last packet sent");
  Udp.beginPacket(server, serverPort);
  Udp.write("bb", 2);
  Udp.endPacket();
  printLog("[info] All data Sent");
  bool no_risp_ok = true;
  while (no_risp_ok){
    char command = receiveMessageFromServer();
    if(command=='a'){
      // No command received
      delay(1000);
    } else {
      if(inPacketBuffer[0]=='3'){
        no_risp_ok = false;
      }
    }
  }
  printLog("[info] Server resp ok");
}
