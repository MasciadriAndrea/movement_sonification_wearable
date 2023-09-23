/*
 *   C4C - TSoftware wearable devices
 *   V: 0.1
 *    
 *  Da eseguire con l'applicazione mobile C4C  
 *  
 *   - Per eseguire in prova sul pc: avviare il programma python main.py
 */
#include <CircularBuffer.h>
#include <PeakDetection.h> // import lib
#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define LED 13
#define LED_RED 12
#define VERBOSE_DATA true
#define VERBOSE_LOG false
#define MAX_DATASET_LEN 10000
//char ssid[] = "FASTWLAN";      // Wifi SSID
//char pass[] = "78E4D6C9BAE78";       // Wifi password
char ssid[] = "c4cHotspot";      // Wifi SSID
char pass[] = "c4cPassword";       // Wifi password
int status = WL_IDLE_STATUS;
//short dataset[MAX_DATASET_LEN][3];
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
PeakDetection peakDetection; // create PeakDetection object
long nt, t, startExe, previous_peak = 0;
CircularBuffer<double,40> doubles;
int ts = 10; // questo parametro è trovato dai dati, non lo si può cambiare

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

  
  //--------------------------peak detector
  peakDetection.begin(100,3,1);//(10, 6, 0.5); // sets the lag, threshold and influence
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
        state = 1;
        startExe = millis();
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
      nt = millis() - startExe;
      long campionamento = nt-t;
      t = nt;
      double data = sqrt(((x) * (x)) + ((y) * (y)) + ((z) * (z)));
      if(dataPosition < MAX_DATASET_LEN){
        datas = short(data*1000);
        dataset[dataPosition] = datas;
        dataPosition += 1;
      }
      peakDetection.add(data); // adds a new data point
      peak = peakDetection.getPeak(); // returns 0, 1 or -1
      double filtered = peakDetection.getFilt(); // moving average
      doubles.push(data);
      double avg = 0;
      double variance = 0;
      for (decltype(doubles)::index_t i = 0; i < doubles.size(); i++) {
          avg += doubles[i] / doubles.size();
      }
      for (decltype(doubles)::index_t i = 0; i < doubles.size(); i++) {
          variance += pow((doubles[i] - avg),2) ;
      }
      variance = variance / doubles.size();
      int peak_m = 0;
      if((data>2.5)&&(peak!=0)&&(variance>0.1)){
        if(nt-previous_peak>300){//2*ts){
          peak_m = 1;
          previous_peak = nt;
        }
      }
      //SEND DATA
      if(peak_m==1){
       printLog("Passo");
       sendpacket(true);
       //delay(50);
      }
      if(VERBOSE_DATA){
        Serial.print(z); // print data
        Serial.print(",");
        Serial.print(data); // print data
        Serial.print(",");
        //Serial.print(z); // print peak status
        //Serial.print(",");
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
        //Serial.print("New message: ");
        //Serial.println(inPacketBuffer);
        //Serial.print("Length: ");
        //Serial.println(packetSize);
      }
      return inPacketBuffer[0];
    }
   return 'a';
}


unsigned long sendpacket(bool peak) {
  //memset(packetBuffer, 0, PACKET_SIZE);
  packetBuffer[0] = peak;
  Udp.beginPacket(server, serverPort);
  Udp.write(packetBuffer, PACKET_SIZE);
  Udp.endPacket();
}

unsigned long send_all_data() {
  char package[9];
  char device = '2';
  int i=0;
  int c=0;
  while(i<dataPosition){
    if(c%10==0) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
    }
    char sensorData[] = {};
    //sprintf(sensorData,"%c%c%u%c%i%c%i%c%i%c",'2',';',i,';',int(dataset[i][0]),';',int(dataset[i][1]),';',int(dataset[i][2])+'\n');  
    //Serial.print(sensorData);
    Udp.beginPacket(server, serverPort);
    //Udp.write((byte*) dataset[i][0], 6);
    memcpy(&package[0],(char *)&device,1);
    memcpy(&package[1],(char *)&i,2);
    //memcpy(&package[3],(char *)&dataset[i][0],2);
    //memcpy(&package[5],(char *)&dataset[i][1],2);
    //memcpy(&package[7],(char *)&dataset[i][2],2);
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
