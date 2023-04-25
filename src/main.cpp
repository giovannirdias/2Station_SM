// Import libraries
#include <Arduino.h>
#include <esp_now.h>
#include <LiquidCrystal.h>
#include <WiFi.h>

// Communication parameters
uint8_t broadcastAddress[] = {};  // Receiver MAC Address
int speed;  // Motor data received
int IRStatus; // IR Sensor data sender

// Structure to message in communication between two esp32
typedef struct message
{
  int value;
}message;

message motorReading;
message irReading;

esp_now_peer_info_t peerInfo;

// Declaration of device
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
#define IR_Sensor 14  // IR sensor pin

// Auxiliar Variables
int IR;    // IR status
int aux = 0;
int box1 = 0, box2 = 0;   // # of boxes passed in conveyor belt     
String message1 = "#white boxes: ";
String message2 = "#black boxes: ";

// Check if data was sent
void CheckDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nStatus do ultimo pacote enviado:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Entrega realizada" : "Falha na entrega");
}

// Check if data was receive
void CheckDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) 
{
  memcpy(&motorReading, incomingData, sizeof(motorReading));
  Serial.print("Bytes recebidos: ");
  Serial.println(len);
  speed = motorReading.value;
}

void lcdMessage(int whitebox, int blackbox)
{
  lcd.print(message1 + String(whitebox));
  lcd.setCursor(0, 1);
  lcd.print(message2 + String(blackbox));
  delay(2000);
  lcd.clear();
}

void setup()
{
  Serial.begin(115200);  // Init Serial Monitor

  // WiFi Station and ESP-NOW Setup
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(CheckDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha ao adicionar o 'peer'");
    return;
  }
  esp_now_register_recv_cb(CheckDataRecv);

  // IR Sensor setup
  pinMode(IR_Sensor, INPUT);

  // LCD Sensor setup
  lcd.begin(16, 2);  // Config. of LCD (#cols, #rows)
  lcd.print("Proj.: Sistemas");
  lcd.setCursor(0, 1);
  lcd.print("Microprocessados");
  delay(10000);
  lcd.clear();
}

void loop()
{

  if (aux == 0)
  {
    lcd.print("Motor Speed now");
    lcd.setCursor(0, 1);
    lcd.print("is " + String(speed) + " RPM");
    delay(10000);
    lcd.clear();
  }
  
  IR=digitalRead(IR_Sensor);
  irReading.value = IR;

  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &irReading, sizeof(irReading));

  if (result == ESP_OK) {
    Serial.println("Enviado com sucesso!");
  }
  else {
    Serial.println("Ocorreu um erro durante o envio.");
  }

  //If sensor detect any reflected ray
  if(IR==LOW)
  {               
    box1++;
    lcdMessage(box1, box2);
  }
  else 
  {
    box2++; 
    lcdMessage(box1, box2);
  }
  aux++;
}
