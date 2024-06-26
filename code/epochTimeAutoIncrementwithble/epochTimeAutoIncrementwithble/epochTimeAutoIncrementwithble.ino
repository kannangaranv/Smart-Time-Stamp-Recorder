#include <math.h>
#include "EEPROM.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_pm.h>

#define BUTTON_PIN1 15
#define BUTTON_PIN2 4
#define BUTTON_PIN3 5
#define DEBOUNCE_DELAY 50  // debounce time in milliseconds
int pinRed = 13; // Red pin
int pinGreen = 14; // Green pin
int pinBlue = 27; // Blue pin

unsigned int timestamp = 1714812148; // Example epoch timestamp (1st Jan 2021 00:00:00)
unsigned int startMillis;
unsigned int millisCorrectionAppToDevice = 1000;

//EEPROM parameters
unsigned short timeStampsPerBtn = 120;
unsigned short eepromPointerAddrbtn1 = 0;//address = 0
unsigned short eepromPointerAddrbtn2 = sizeof(unsigned short); //address = 2
unsigned short eepromPointerStartBtn1 = sizeof(unsigned short)*2; //address = 4
unsigned short eepromPointerEndBtn1 = eepromPointerStartBtn1 + sizeof(unsigned int) * (timeStampsPerBtn-1); //address = 480
unsigned short eepromPointerStartBtn2 = sizeof(unsigned short)*2 + sizeof(unsigned int) * (timeStampsPerBtn + 1); //address = 488
unsigned short eepromPointerEndBtn2 = eepromPointerStartBtn2 + sizeof(unsigned int) * (timeStampsPerBtn-1); //address = 964
unsigned short eepromPointerAddrTimestampSave = eepromPointerEndBtn1 + 4;
unsigned short eepromPointerAddrGarbage = eepromPointerEndBtn2 + 4;

unsigned short eepromPointer;
unsigned int button1Data[120];
unsigned int button2Data[120];

// Variables for button 1
int lastState1 = HIGH;
int currentState1;
unsigned long pressStartTime1 = 0;
int indxBtn1;

// Variables for button 2
int lastState2 = HIGH;
int currentState2;
unsigned long pressStartTime2 = 0;
int indxBtn2;

// Variables for button 3
int lastState3 = HIGH;
int currentState3;
unsigned long pressStartTime3 = 0;

//Bluetooth ------------------------------------------------------------------------------
//Variables for Ble
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CUSTOM_CHAR_UUID    "beefcafe-36e1-4688-b7f5-00000000000b"
#define BUTTON1_CHAR_UUID   "deadbeef-36e1-4688-b7f5-000000000001"
#define BUTTON2_CHAR_UUID   "cafebeef-36e1-4688-b7f5-000000000002"

BLEServer* pServer = nullptr;
BLECharacteristic* pCustomCharacteristic = nullptr;
BLECharacteristic* pButton1Characteristic = nullptr;
BLECharacteristic* pButton2Characteristic = nullptr;

bool isBleDisabled = true;
void bluetoothTurnOff();

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      Serial.println();
      Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println();
      Serial.println("Device disconnected");
      bluetoothTurnOff();
      isBleDisabled = true;
    }
};
unsigned int* sortArrayDescending(const unsigned int inputArray[], size_t arraySize);

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value == "get/button1/1-30") {
          sendButtonData(pButton1Characteristic, sortArrayDescending(button1Data,timeStampsPerBtn),1);
          Serial.println("Button1 data sent to app");
      } else if (value == "get/button1/31-60") {
          sendButtonData(pButton1Characteristic, sortArrayDescending(button1Data,timeStampsPerBtn),31);
          Serial.println("Button1 data sent to app");
      }else if (value == "get/button1/61-90") {
          sendButtonData(pButton1Characteristic, sortArrayDescending(button1Data,timeStampsPerBtn),61);
          Serial.println("Button1 data sent to app");
      }else if (value == "get/button1/91-120") {
          sendButtonData(pButton1Characteristic, sortArrayDescending(button1Data,timeStampsPerBtn),91);
          Serial.println("Button1 data sent to app");

      }else if (value == "get/button2/1-30") {
          sendButtonData(pButton2Characteristic, sortArrayDescending(button2Data,timeStampsPerBtn),1);
          Serial.println("Button2 data sent to app");
      }else if (value == "get/button2/31-60") {
          sendButtonData(pButton2Characteristic, sortArrayDescending(button2Data,timeStampsPerBtn),31);
          Serial.println("Button2 data sent to app");
      }else if (value == "get/button2/61-90") {
          sendButtonData(pButton2Characteristic, sortArrayDescending(button2Data,timeStampsPerBtn),61);
          Serial.println("Button2 data sent to app");
      }else if (value == "get/button2/91-120") {
          sendButtonData(pButton2Characteristic, sortArrayDescending(button2Data,timeStampsPerBtn),91);
          Serial.println("Button2 data sent to app");
      }else if (value.length()==10){
          unsigned int intValue = strtoul(value.c_str(), nullptr, 10);
          Serial.println(intValue);  // Print the converted integer value
          startMillis = millis()- millisCorrectionAppToDevice;
          timestamp = intValue;

      }
      
    }

    void sendButtonData(BLECharacteristic *pCharacteristic, unsigned int* data, int start) {
        for (int i = start-1; i < start+29; i++) {
            char buffer[11]; // To hold the string representation of each number
            snprintf(buffer, 11, "%010u", data[i]); // Convert the number to string with fixed length
            pCharacteristic->setValue(buffer); // Set the fixed length
            pCharacteristic->notify();
            delay(3); // Adjust delay as needed to ensure data is received properly
        }
    }
};

void bluetoothConnectionManagement() {
  if(isBleDisabled){
    isBleDisabled = false;
    digitalWrite(pinGreen,HIGH);
    setCpuFrequencyMhz(240);
    disableModemSleep();

    Serial.println(); Serial.println("Starting BLE work!");
    BLEDevice::init("ESP32_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Security setup
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setStaticPIN(123456); // Set your desired static PIN here

    pCustomCharacteristic = pService->createCharacteristic(
      CUSTOM_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCustomCharacteristic->addDescriptor(new BLE2902());
    pCustomCharacteristic->setCallbacks(new MyCallbacks());

    // Button1 characteristic
    pButton1Characteristic = pService->createCharacteristic(
      BUTTON1_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_INDICATE
    );
    pButton1Characteristic->addDescriptor(new BLE2902());

    // Button2 characteristic
    pButton2Characteristic = pService->createCharacteristic(
      BUTTON2_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_INDICATE
    );
    pButton2Characteristic->addDescriptor(new BLE2902());

    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    Serial.println("Waiting for a client connection to notify...");
  }

}

void bluetoothTurnOff() {
  if(!isBleDisabled){
    isBleDisabled = true;
    digitalWrite(pinGreen,LOW);
    Serial.println();Serial.println("Stopping BLE advertising...");
    // Stop advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->stop();
    // Optionally, you can also shut down the BLE device completely
    BLEDevice::deinit(true);
    Serial.println("BLE advertising stopped and device turned off.");
    //Low level turn off
    enableModemSleep();
    setCpuFrequencyMhz(240);
  }
}

//Bluetooth End -----------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN3, INPUT_PULLUP);
  pinMode(pinRed, OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue, OUTPUT);

  startMillis = millis();  // Get the current time

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  //Update each arrays of buttons
  epochGetFromEEPROM(0);
  epochGetFromEEPROM(1);

  recoverRestartImmediately();
  bluetoothConnectionManagement();
}

void loop() {
  buttonPressingPatternManage();
  if(isBleDisabled){
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Sleep for 50 mili second
  }
}

void buttonPressingPatternManage(){
  //Check button 1
  currentState1 = digitalRead(BUTTON_PIN1);
  if (currentState1 != lastState1) {
    delay(DEBOUNCE_DELAY);
    currentState1 = digitalRead(BUTTON_PIN1);
  }
  if (lastState1 == HIGH && currentState1 == LOW) {
    pressStartTime1 = millis();
  } 
  else if (lastState1 == LOW && currentState1 == HIGH) {
    if (millis() - pressStartTime1 >= 100) {
      Serial.println(" ");
      unsigned int updatedEpoch = calculateEpoch();
      epochWriteToEEPROM(0,updatedEpoch);
    } 
  }
  lastState1 = currentState1;

  // Check button 2
  currentState2 = digitalRead(BUTTON_PIN2);
  if (currentState2 != lastState2) {
    delay(DEBOUNCE_DELAY);
    currentState2 = digitalRead(BUTTON_PIN2);
  }
  if (lastState2 == HIGH && currentState2 == LOW) {
    pressStartTime2 = millis();
  }
  else if (lastState2 == LOW && currentState2 == HIGH) {
    if (millis() - pressStartTime2 >= 100) {
      Serial.println(" ");
      unsigned int updatedEpoch = calculateEpoch();
      epochWriteToEEPROM(1,updatedEpoch);
    }
  }
  lastState2 = currentState2;

  // Check button 3
  currentState3 = digitalRead(BUTTON_PIN3);
  if (currentState3 != lastState3) {
    delay(DEBOUNCE_DELAY);
    currentState3 = digitalRead(BUTTON_PIN3);
  }
  if (lastState3 == HIGH && currentState3 == LOW) {
    pressStartTime3 = millis();
  }
  else if (lastState3 == LOW && currentState3 == HIGH) {
    if (millis() - pressStartTime3 >= 1200) {
      if(isBleDisabled){
        restartImmediately(); 
      }
    } else {
      bluetoothTurnOff();
    }
  }
  lastState3 = currentState3;
}


  unsigned int calculateEpoch() {
    unsigned int currentMillis = millis();  // Get the current time
    unsigned int durationSeconds = round((currentMillis - startMillis)/1000);
    Serial.print("duration: "); Serial.println(durationSeconds);
    return (timestamp+durationSeconds);
  }


  void epochWriteToEEPROM(bool button, unsigned int epochTimestamp ){
    if(!button){
      eepromPointer = EEPROM.readUShort(eepromPointerAddrbtn1);
      if(eepromPointer > eepromPointerEndBtn1){
        eepromPointer = eepromPointerStartBtn1;
      }
      indxBtn1 = eepromPointer / 4;
      Serial.print("Address write "); Serial.print(indxBtn1);Serial.print(" : "); Serial.println(eepromPointer);
      Serial.print("Timestamp for button 1: "); Serial.println(epochTimestamp);
      EEPROM.writeUInt(eepromPointer, epochTimestamp);
      eepromPointer += sizeof(unsigned int);
      EEPROM.writeUShort(eepromPointerAddrbtn1,eepromPointer);
      EEPROM.commit();  
      button1Data[indxBtn1-1] = epochTimestamp;
    }
    else{
      eepromPointer = EEPROM.readUShort(eepromPointerAddrbtn2);
      if(eepromPointer > eepromPointerEndBtn2){
        eepromPointer = eepromPointerStartBtn2;
      }
      indxBtn2 = (eepromPointer - eepromPointerStartBtn2 + 4) / 4;
      Serial.print("Address write "); Serial.print(indxBtn2);Serial.print(" : "); Serial.println(eepromPointer);
      Serial.print("Timestamp for button 2: "); Serial.println(epochTimestamp);
      EEPROM.writeUInt(eepromPointer, epochTimestamp);
      eepromPointer += sizeof(unsigned int);
      EEPROM.writeUShort(eepromPointerAddrbtn2,eepromPointer);
      EEPROM.commit(); 
      button2Data[indxBtn2-1] = epochTimestamp;
    }
      Serial.print("CPU Speed: ");Serial.print(ESP.getCpuFreqMHz());Serial.println(" MHz");
  }


  void epochGetFromEEPROM(bool button){
    unsigned short i = 0;
    if(!button){
      unsigned short address = eepromPointerStartBtn1;
      while(i <= timeStampsPerBtn){
        button1Data[i]=EEPROM.readUInt(address);
        address += sizeof(unsigned int);
        i++;
      }
    }
    else{
      unsigned short address = eepromPointerStartBtn2;
      while(i <= timeStampsPerBtn){
        button2Data[i]=EEPROM.readUInt(address);
        address += sizeof(unsigned int);
        i++;
      }
    }
  }


  void printArray(unsigned int arr[], int size) {
    Serial.println();
    for (int i = 0; i < size; i++) {
      Serial.print(i+1);
      Serial.print(":");
      Serial.print(arr[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

// Function to compare two integers for descending order
int compareDescending(const void* a, const void* b) {
  unsigned int arg1 = *(const unsigned int*)a;
  unsigned int arg2 = *(const unsigned int*)b;
  
  if (arg1 < arg2) return 1;
  if (arg1 > arg2) return -1;
  return 0;
}

// Function that sorts an array in descending order and returns a new array
unsigned int* sortArrayDescending(const unsigned int inputArray[], size_t arraySize) {
  // Allocate memory for a new array that will be returned
  unsigned int* newArray = (unsigned int*)malloc(arraySize * sizeof(unsigned int));
  if (newArray == NULL) {
    Serial.println("Memory allocation failed");
    return NULL; // Return NULL if memory allocation fails
  }

  // Copy the elements of the input array to the new array
  memcpy(newArray, inputArray, arraySize * sizeof(unsigned int));

  // Sort the new array in descending order
  qsort(newArray, arraySize, sizeof(unsigned int), compareDescending);

  return newArray;
}

void restartImmediately(){
  timestamp = calculateEpoch();
  timestamp+=1;
  EEPROM.writeUInt(eepromPointerAddrTimestampSave, timestamp);
  EEPROM.writeUInt(eepromPointerAddrGarbage, 1);
  EEPROM.commit();  
  Serial.println("");Serial.println(timestamp);Serial.println("Timestamp Saved! Restarting...");
  ESP.restart();
}

void recoverRestartImmediately(){
  startMillis = millis();
  timestamp = EEPROM.readUInt(eepromPointerAddrTimestampSave);
  Serial.println("");Serial.println(timestamp);Serial.println("Timestamp Recovered!");
}

void enableModemSleep() {
    esp_bt_controller_disable();
    Serial.println("Modem sleep mode enabled");
}

void disableModemSleep() {
    // Enable Bluetooth
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    Serial.println("Modem sleep mode disabled");
}
