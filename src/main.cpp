/*
TODO:

//??- When a new track is selected, the goalpoint isnt reset so it moves to the point anyways
//?- Add deadzone in middle
//?- Add file -temp download for interruptions
//- File download progress

*/

#include <Arduino.h>

#include <TMCStepper.h>

#include <AccelStepper.h>

#include <MultiStepper.h>

#include <ArduinoJson.h>

#include <WiFi.h>

#include <NimBLEDevice.h>

// #include <NimBLE2902.h>

#include <LittleFS.h>

#include <FirebaseClient.h>

#include <SD.h>

#include <WiFiClientSecure.h>


#define API_KEY "AIzaSyA-mjTxBQ87e6EfIMScxcevufxP11UukMs"
#define FIREBASE_PROJECT_ID "tottori-28f47"
#define USER_EMAIL "tottorimain@gmail.com"
#define USER_PASSWORD "Yabadabadoo9000!Tottori"
#define STORAGE_BUCKET_ID "tottori-28f47.appspot.com"

void authHandler();

void printResult(AsyncResult &aResult);

void printError(int code, const String &msg);

DefaultNetwork network; // initilize with boolean parameter to enable/disable network reconnection

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);

FirebaseApp app;

WiFiClientSecure ssl_client;

using AsyncClient = AsyncClientClass;

AsyncClient aClient(ssl_client, getNetwork(network));

Firestore::Documents Docs;

AsyncResult aResult_no_callback;

int counter = 0;

unsigned long dataMillis = 0;

bool taskCompleted = false;

double downloadProgress = -1;

void authHandler();

void printResult(AsyncResult &aResult);

void printError(int code, const String &msg);

void fileCallback(File &file, const char *filename, file_operating_mode mode);

Storage storage;

NimBLEServer *pServer = NULL;
NimBLEService *pService = NULL;
NimBLECharacteristic *pWiiFiCredCharacteristic = NULL;
NimBLECharacteristic *pStatusCharacteristic = NULL;
NimBLECharacteristic *pRemoteCharacteristic = NULL;
// Format: (0(paused) / 1(playing)) + 0x1D + (Q(queue) / T(track)) + UUID + 0x1D (1.3.2 (nested queue index))
NimBLEAdvertisementData advertisementData = NimBLEAdvertisementData();

#define SERVICE_UUID        "96209393-38a2-4740-9b09-467da594a13c"
#define WIFI_CREDENTIAL_CHARACTERISTIC_UUID "42b7c22e-009c-43e6-a356-d7264f2a5ec5"
#define STATUS_CHARACTERISTIC_UUID "a9c0220b-1b06-48e7-a563-71731910a244"
#define REMOTE_CHARACTERISTIC_UUID "0f4e1e4c-0a50-4f6e-8569-1fef7bb657b4"

StaticJsonDocument<1536> tableInfoDoc;

//Higher subdivisions mean higher accuracy in linear interpolation, but also slower performance. 
//Units in line cuts per `1` unit distance
#define subdivisions 100.0
#define microstep 8
int angSpeed = 1500;
int distSpeed = 1500;

//345/20 tooth gear
#define bigger_gear 345
#define smaller_gear 20
#define axis_length 920
int stepsPerRotation = (200 / smaller_gear) * microstep * bigger_gear;
int axisSteps = axis_length * microstep;
unsigned long waitEnd = 0;
#define CENTER_DEADZONE 0.1

#define limitSwitchPin 15
#define WIFI_TIMEOUT 5000 // Wi-Fi connection timeout (ms)
#define WIFI_PREFERENCE_TIMEOUT 10000 // Wi-Fi preference connection timeout (ms)

String wifiPreference = "";
bool endWifiAutoConnect = false;

////////=====ANGLE MOTOR DEFINITIONS====////////

    #define DRIVER_ADDRESS       0b00   // TMC2209 Driver address according to MS1 and MS2
    #define R_SENSE              0.11f  // SilentStepStick series use 0.11

    #define ANG_EN_PIN           33     // Enable
    #define ANG_DIR_PIN          2      // Direction
    #define ANG_STEP_PIN         4      // Step
    #define ANG_SW_SCK           32     // Software Slave Clock (SCK)
    #define ANG_SERIAL_PORT      Serial1

    //SoftwareSerial AngleSoftSerial(ANG_SW_RX, ANG_SW_TX);
    //TMC2209Stepper AngleTMCDriver(&AngleSoftSerial, R_SENSE, DRIVER_ADDRESS);
    TMC2209Stepper angleTMCDriver(&ANG_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
    AccelStepper angleStepper = AccelStepper(1, ANG_STEP_PIN, ANG_DIR_PIN);

////////=====DISTANCE MOTOR DEFINITIONS====////////

    #define DIST_EN_PIN           26     // Enable
    #define DIST_DIR_PIN          27     // Direction
    #define DIST_STEP_PIN         13     // Step
    #define DIST_SW_SCK           21     // Software Slave Clock (SCK)
    #define DIST_SERIAL_PORT      Serial2

    //SoftwareSerial DistSoftSerial(DIST_SW_RX, DIST_SW_TX);
    //TMC2209Stepper DistTMCDriver(&DistSoftSerial, R_SENSE, DRIVER_ADDRESS);
    TMC2209Stepper distTMCDriver(&DIST_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
    AccelStepper distStepper = AccelStepper(1, DIST_STEP_PIN, DIST_DIR_PIN);

    MultiStepper steppers;

bool isPlaying = false;
bool isTrackSelected = false;
// String pendingQueueStack = "";
String pendingUid = "";
String currentUid = "";
int queueIndex = 0;
bool isLooping = true;
bool isShuffle = false;
int queueLength = 0;
// String queueStack;
String queue = "";
File currentTrack;
size_t trackPosition;
double trackProgress = 0;
int lineProgress = 0;

void moveMotorsTowards(int angle, int distance) {
  int currentAngle = angleStepper.currentPosition();
  if ((currentAngle - angle) > stepsPerRotation/2) {
    Serial.println("Looping over 1, set angleStepper to: " + String(angleStepper.currentPosition()-stepsPerRotation));
    angleStepper.setCurrentPosition(currentAngle - stepsPerRotation);
  } else if ((angle - currentAngle) > stepsPerRotation/2) {
    Serial.println("Looping over 2, set angleStepper to: " + String(angleStepper.currentPosition()+stepsPerRotation));
    angleStepper.setCurrentPosition(currentAngle + stepsPerRotation);
  }
  if (distance > axisSteps) {
    Serial.println("!!!!!!!!! ERROR: DISTANCE MOTOR POS GREATER THAN MAX !!!!!!!!!");
    distance = axisSteps;
  } else if (distance < 0) {
    Serial.println("!!!!!!!!! ERROR: DISTANCE MOTOR POS LESS THAN ZERO !!!!!!!!!");
    distance = 0;
  }
  long positions[2];
  // if (angle > CENTER_DEADZONE) {
  //   positions[0] = angleStepper.currentPosition();
  // } else {
    positions[0] = angle;
  // }
  positions[1] = distance;
  //Serial.println(", Motor Positions: (" + String(angle) + ", " + String(distance) + ")");
  steppers.moveTo(positions);
  return;
}

class CartesianPoint; // Forward declaration

class PolarPoint {

  public:
    double angle;
    double distance;

    String toString() {
        return (String("(") + String(angle) + String(", ") + String(distance) + String(")"));
    }

    int getAngleMotorPos() {
        return (angle/(2*PI)) * stepsPerRotation;
    }

    int getDistanceMotorPos() {
        return distance * axisSteps;
    }

    PolarPoint(double nAngle, double nDistance) {
        angle = nAngle;
        distance = nDistance;
    }

    PolarPoint(CartesianPoint other); // Forward declaration of the constructor

};

class CartesianPoint {

  public:
    bool exists = true;
    double x;
    double y;

    String toString() {
        return (String("(") + String(x) + String(", ") + String(y) + String(")"));
    }

    CartesianPoint lerp(CartesianPoint other, double t) {
        t = min(1.0, max(0.0, t));
        return CartesianPoint((1 - t) * x + t * other.x, (1 - t) * y + t * other.y);
    }

    double distanceTo(CartesianPoint other) {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }

    void update(double nX, double nY) {
        x = nX;
        y = nY;
    }

    CartesianPoint(double nX, double nY) {
        x = nX;
        y = nY;
    }

    CartesianPoint() {
        exists = false;
    }

    void update(PolarPoint other) {
        x = cos(other.angle) * other.distance;
        y = sin(other.angle) * other.distance;
    }

};

// Definition of the constructor
PolarPoint::PolarPoint(CartesianPoint other) {
    angle = atan2(other.y, other.x);
    distance = sqrt(pow(other.x, 2) + pow(other.y, 2));
}


CartesianPoint point1 = CartesianPoint(0,0);
CartesianPoint point2 = CartesianPoint(0,0);
CartesianPoint remotePoint1 = CartesianPoint();
CartesianPoint remotePoint2 = CartesianPoint();
PolarPoint goalPoint = PolarPoint(0,0);

File openTableInfo() {

  File tableInfo = SD.open("/tableInfo.json", FILE_READ, false);

  if (!tableInfo || !SD.exists("/tableInfo.json")) {
    Serial.println("No tableInfo.json!");
    tableInfoDoc.clear();
    tableInfoDoc["name"].set("My Table");
    tableInfoDoc["angSpeed"].set("1500");
    tableInfoDoc["distSpeed"].set("1500");
    tableInfoDoc.createNestedArray("networks");
    tableInfo.close();
    tableInfo = SD.open("/tableInfo.json", FILE_WRITE, true);
    serializeJson(tableInfoDoc,tableInfo);
    tableInfo.close();
    tableInfo = SD.open("/tableInfo.json", FILE_READ, false);
  } else {
    deserializeJson(tableInfoDoc, tableInfo);
    tableInfo.close();
    if (!tableInfoDoc.containsKey("name")) {
      tableInfoDoc["name"].set("My Table");
    }
    if (!tableInfoDoc.containsKey("angSpeed")) {
      tableInfoDoc["angSpeed"].set(1500);
    }
    if (!tableInfoDoc.containsKey("distSpeed")) {
      tableInfoDoc["distSpeed"].set(1500);
    }
    if (!tableInfoDoc.containsKey("networks")) {
      tableInfoDoc.createNestedArray("networks");
    }
    tableInfo = SD.open("/tableInfo.json", FILE_WRITE, true);
    serializeJson(tableInfoDoc,tableInfo);
    tableInfo.close();
    tableInfo = SD.open("/tableInfo.json", FILE_READ, false);
  }
  
  tableInfoDoc.clear(); //redundant?
  return tableInfo;
}

void updateStatus() {

  String statusValue = "";
  
  statusValue += WiFi.isConnected() ? "1" : "0";  //0
  statusValue += char(0x1D);
  statusValue += WiFi.localIP().toString();       //1
  statusValue += char(0x1D);
  statusValue += isPlaying ? "1" : "0";           //2
  statusValue += char(0x1D);
  statusValue += isTrackSelected ? "T" : "Q";     //3
  statusValue += char(0x1D);
  statusValue += currentUid.substring(1);         //4
  statusValue += char(0x1D);
  statusValue += queue;                           //5
  statusValue += char(0x1D);
  statusValue += queueIndex;                      //6
  statusValue += char(0x1D);
  statusValue += isLooping ? "1" : "0";           //7
  statusValue += char(0x1D);
  statusValue += isShuffle ? "1" : "0";           //8
  statusValue += char(0x1D);
  statusValue += queueLength;                     //9
  statusValue += char(0x1D);
  statusValue += trackProgress;                   //10
  statusValue += char(0x1D);
  statusValue += angSpeed;                        //11
  statusValue += char(0x1D);
  statusValue += distSpeed;                       //12
  statusValue += char(0x1D);
  statusValue += downloadProgress;                //13
  
  Serial.println("Updating status to: " + statusValue);

  pStatusCharacteristic->setValue(statusValue);

  pStatusCharacteristic->notify();

}

void asyncCB(AsyncResult &aResult) {
  printResult(aResult);
  if (aResult.isError()) {
    downloadProgress = -1;
  } else if (aResult.downloadProgress()) {
    downloadProgress = aResult.downloadInfo().downloaded/(double)aResult.downloadInfo().total;
    updateStatus();
  }    
};

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected");
        Serial.println("Multi-connect support: start advertising");
        NimBLEDevice::startAdvertising();
    };
    /** Alternative onConnect() method to extract details of the connection.
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */
    // void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    //     Serial.print("Client address: ");
    //     Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    //     /** We can use the connection handle here to ask for different connection parameters.
    //      *  Args: connection handle, min connection interval, max connection interval
    //      *  latency, supervision timeout.
    //      *  Units; Min/Max Intervals: 1.25 millisecond increments.
    //      *  Latency: number of intervals allowed to skip.
    //      *  Timeout: 10 millisecond increments, try for 5x interval time for best results.
    //      */
    //     pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    // };
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };
};

void addWifiCredentials(String ssid, String pass) {

  Serial.println("Adding Wifi Credentials --- SSID: \"" + ssid + "\" PASS: \"" + pass + "\"");
  File tableInfo = openTableInfo();
  deserializeJson(tableInfoDoc, tableInfo);

  // If tableInfo already contains ssid, set new pass for ssid instead of creating a new array
  for (JsonArray network : tableInfoDoc["networks"].as<JsonArray>()) {
    if (ssid.equals(network[0].as<const char*>())) {
      network[1].set(pass.c_str());
      Serial.println("New json (non-append):");
      serializeJsonPretty(tableInfoDoc, Serial);
      tableInfo.close();
      tableInfo = SD.open("/tableInfo.json", FILE_WRITE);
      serializeJson(tableInfoDoc, tableInfo);
      tableInfoDoc.clear();
      return;
    }
  }
  JsonArray newCredential = tableInfoDoc["networks"].createNestedArray();
  newCredential.add(ssid.c_str());
  newCredential.add(pass.c_str());
  Serial.println("New json:");
  serializeJsonPretty(tableInfoDoc, Serial);
  tableInfo.close();
  tableInfo = SD.open("/tableInfo.json", FILE_WRITE);
  serializeJson(tableInfoDoc, tableInfo);
  tableInfoDoc.clear();
}

void stopWifiAutoconnect() {
  endWifiAutoConnect = true;
  while (endWifiAutoConnect == true)
  return;
}

void loadSettings() {

  File tableInfo = openTableInfo();
  deserializeJson(tableInfoDoc, tableInfo);
  if (tableInfoDoc.containsKey("angSpeed")) {
      angSpeed =  tableInfoDoc["angSpeed"];
      angleStepper.setSpeed(angSpeed);
    }
    if (tableInfoDoc.containsKey("distSpeed")) {
      distSpeed = tableInfoDoc["distSpeed"];
      distStepper.setSpeed(distSpeed);
    }
    updateStatus();
}

void wifiAutoconnect(String ssidPreference = "") {
 
  File tableInfo = openTableInfo();
  deserializeJson(tableInfoDoc, tableInfo);

  Serial.println("ssidPrefernce: \"" + ssidPreference + "\" isEmpty: " + ssidPreference.isEmpty());
  if (!ssidPreference.isEmpty()) {
    for (JsonArray network : tableInfoDoc["networks"].as<JsonArray>()) {
      if (ssidPreference.equals(network[0].as<const char*>())) {
        WiFi.begin(network[0].as<const char*>(), network[1].as<const char*>());
        long start = millis(); 
        while (WiFi.status() != WL_CONNECTED && millis()-WIFI_PREFERENCE_TIMEOUT < start) {
          Serial.print(".");
          delay(300);
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("Preference connected to --- SSID: \"" + String(network[0].as<const char*>()) + "\" PASS: \"" + String(network[1].as<const char*>()) + "\"");
          wifiPreference = "";
          updateStatus();
          return;
        } else {
          WiFi.disconnect();
        }
      }
    }
  }

  while (WiFi.status() != WL_CONNECTED) {

    if (endWifiAutoConnect == true) {
      endWifiAutoConnect = false;
      return;
    }
    
    int numNetworks = WiFi.scanNetworks();
    int networksRSSI[numNetworks];
    int networksInOrder[numNetworks];
    for (int i = 0; i<numNetworks; i++) {
      networksRSSI[i] = WiFi.RSSI(i);
    }
    //Sort networks by RSSI
    Serial.println("------------------");
    for (int i = 0; i<numNetworks; i++) {
      int maxIndex = 0;
      for (int j = 1; j<numNetworks; j++) {
        if (networksRSSI[maxIndex] < networksRSSI[j]) {
          maxIndex = j;
        }
      }
      networksRSSI[maxIndex] = INT16_MIN;
      networksInOrder[i] = maxIndex;
      Serial.print("SSID: ");
      Serial.print(WiFi.SSID(maxIndex));
      Serial.print(", RSSI: ");
      Serial.println(WiFi.RSSI(maxIndex));
    }
    Serial.println("---");

    for (int i= 0; i<numNetworks; i++) {
      Serial.println(tableInfoDoc["networks"].as<JsonArray>());
      for (JsonArray network : tableInfoDoc["networks"].as<JsonArray>()) {
        Serial.println("-------");
        Serial.println("Trying to connect to " + String(network[0].as<const char*>()) + " : " + String(network[1].as<const char*>()));
        if (WiFi.SSID(networksInOrder[i]).equals(network[0].as<const char*>())) {
          Serial.println("Connecting to " + String(network[0].as<const char*>()) + " : " + String(network[1].as<const char*>()));
          WiFi.begin(network[0].as<const char*>(), network[1].as<const char*>());
          long start = millis(); 
          while (WiFi.status() != WL_CONNECTED && millis()-WIFI_TIMEOUT < start) {
            Serial.print(".");
            delay(300);
          }
          if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            Serial.print("Connected with IP: ");
            Serial.println(WiFi.localIP());
            Serial.println();
            updateStatus();
            return;
          } else {
            WiFi.disconnect();
          }
        }
      } 
    }   
    Serial.println("------------------");

  }

    // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Serial.print("Connecting to Wi-Fi");
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     Serial.print(".");
    //     delay(300);
    // }
    

}





bool setCurrentTrackCallbackHelper(String uid, int tries) {

Serial.print("Setting current track to: ");
  Serial.println(uid.c_str());

  if (currentTrack) {
    String currentTrackName = String(currentTrack.name());
    if (currentTrackName.substring(0, currentTrackName.indexOf('.')).equals(uid.c_str())) {
      return true;
    }
    currentTrack.close();
  }
  lineProgress = 0;
  currentTrack.close();
  String path = "tots/" + uid + ".tot";
  if (SD.exists("/" + String(path.c_str()))) {
    Serial.print("Set current track to (already existed): ");
    Serial.println(uid.c_str());
    currentTrack = SD.open("/"+String(path.c_str()), FILE_READ);
    Serial.print("It is this long: ");
    Serial.println(currentTrack.size());
    trackPosition = 0;
    trackProgress = 0;
    return true;
  }
  Serial.print("Downloading to path: ");
  String downloadPath = "/" + path.substring(0,path.lastIndexOf(".")) + "-temp.tot";
  Serial.println(downloadPath);
  if (SD.exists(downloadPath)) {
    SD.remove(downloadPath);
  }
  //TODO: replace w/ just downloadPath i havent tested if it works and im too tired to care
  FileConfig download_file = FileConfig(path.substring(0,path.lastIndexOf(".")) + "-temp.tot", fileCallback);
  storage.download(aClient, FirebaseStorage::Parent(STORAGE_BUCKET_ID, path), getFile(download_file), asyncCB, "downloadTask");
  SD.rename(downloadPath, "/" + path.substring(0,path.lastIndexOf(".")) + ".tot");
  downloadProgress = -1;
  Serial.print("Set current track to (downloaded): ");
  Serial.println(uid.c_str());
  currentTrack = SD.open("/" + String(path.c_str()), FILE_READ);
  Serial.print("It is this long: ");
  Serial.println(currentTrack.size());
  trackPosition = 0;
  trackProgress = 0;
  return true;
  
}

bool setCurrentTrack(String uid) {

  int maxTries = 10;
  return setCurrentTrackCallbackHelper(uid, maxTries);
}

// Get track uid from index in queue
String getTrackAtIndex(int index) {
  
  String queueCopy = queue + ".";
  for (int i = 0; i < index; i++) {
    queueCopy = queueCopy.substring(queueCopy.indexOf('.')+1);
    // int nextSearchIndex = queue.indexOf('.',searchIndex);
    // if (nextSearchIndex == -1) {
    //   break; 
    // } else {
    //   searchIndex = nextSearchIndex+1;
    // }
  }
  return queueCopy.substring(0,queueCopy.indexOf('.'));
  // int lastSearchIndex = queue.indexOf('.',searchIndex+1);
  // if (lastSearchIndex == -1) {
  //   return queue.substring(searchIndex);
  // } else {
  //   return queue.substring(searchIndex,lastSearchIndex);
  // }
}

// Sets current queue from UID
bool setCurrentQueue(String uid) {
  String payload = Docs.get(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "/queues/" + uid, GetDocumentOptions(DocumentMask("children")));
  if (aClient.lastError().code() == 0) {
    Serial.println(payload);
    DynamicJsonDocument doc(3072);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return false;
    }
    // FirebaseJson json;
    // FirebaseJsonArray arr;
    // FirebaseJsonData result;
    // json.setJsonData(payload);
    // json.get(result, "fields/children/arrayValue/values");
    // if (result.success) {
    if(doc.containsKey("children")) {
      if (!doc["children"].as<JsonArray>().isNull()) {
        // result.getArray(arr);
        // size_t max = arr.iteratorBegin();
        int count = 0;
        queue = "";
        JsonArray array = doc["children"].as<JsonArray>();
        for(JsonVariant v : array) {
          
          if (!v.isNull()) {
            String value = v.as<String>();
            if (value.charAt(1) == 'T') {
              queue += (value.substring(1,value.length()-1).c_str());
              queue += ".";
              count++;
            } else if (value.charAt(1) == 'W') {
              queue += (value.substring(1,value.length()-1).c_str());
              queue += ".";
              count++;
            } 
          }
        }
        queueLength = count;
        queue = queue.substring(0,queue.length()-1);
        Serial.println("im done! ^_^");
        Serial.println(queue);
        Serial.println(queueLength);
        for (int i = 0; i < queueLength; i++) {
          Serial.println(i);
          Serial.println(getTrackAtIndex(i));
        }
        return true;
      }
      return false;
    }  
    return false;
  } else {
    printError(aClient.lastError().code(), aClient.lastError().message());
    return false;
  }
}

void next() {
  if (isTrackSelected) {
    isPlaying = isLooping;
    trackPosition = 0; 
    return;
  }
  queueIndex++;
  isPlaying = true;
  if (isLooping) {
    queueIndex%=queueLength;
  } else if (queueIndex == queueLength) {
    queueIndex=0;
    isPlaying = false;
  }
  String nextTrack = getTrackAtIndex(queueIndex);
  currentUid = nextTrack;
  if (nextTrack.charAt(0) == 'W') {
    waitEnd = millis() + nextTrack.substring(1).toInt();
    currentTrack.close();
  } else if (nextTrack.charAt(0) == 'T') {
    setCurrentTrack(nextTrack);
  } else {
    next();
  }
}

class WifiCredentialCallback: public NimBLECharacteristicCallbacks {

	void onWrite(NimBLECharacteristic *pCharacteristic) {

	  // home wifi: 4154545058776A411D2B377A6D326B623F73382566
    // school wifi: 57696C646361745F4775657374731D77696C646361746775657374
    stopWifiAutoconnect();
		String data = pCharacteristic->getValue().c_str();
    Serial.println(String("Wifi data recieved: ") + String(data));
    int seperatorIndex = data.indexOf(0x1D);
    String ssid=data;
    if (seperatorIndex != -1) {
      ssid = data.substring(0,seperatorIndex);
      String pass = data.substring(seperatorIndex+1);
      addWifiCredentials(ssid, pass);
    }
    wifiPreference = ssid;
    WiFi.disconnect();
    updateStatus();
   
	}

};

class RemoteCallback: public NimBLECharacteristicCallbacks {

  //  Format:
  //  isPlaying (0/1) 0x1D (T/Q)UID 0x1D queueIndex 0x1D isLooping (0/1) 0x1D isShuffle (0/1)
  

  // Pause
  // 301D5135333036626362322D633031392D343331632D613461392D6366353232623438323836661D321D301D30
  // Play at (2)
  // 311D5135333036626362322D633031392D343331632D613461392D6366353232623438323836661D321D301D30
  // Play at end (4)
  // 311D5135333036626362322D633031392D343331632D613461392D6366353232623438323836661D341D301D30
  // Play w/ loop
  // 311D5135333036626362322D633031392D343331632D613461392D6366353232623438323836661D321D311D30
  // Play w/ shuffle
  // 311D5135333036626362322D633031392D343331632D613461392D6366353232623438323836661D321D301D31
	void onWrite(NimBLECharacteristic *pCharacteristic) {

    String data = String(pCharacteristic->getValue().c_str());
    data.concat(char(29));
    int index = data.indexOf(29);
    int i = 0;
    bool hasX = false;
    double tempX;
    while (index != -1) {
      String section = data.substring(0,index);
      if (section.length() > 0 && section != "null") {
        switch (i) {
          case 0:
            isPlaying = section.charAt(0) == '1';
            break;
          case 1:
            isTrackSelected = section.charAt(0) == 'T';
            pendingUid = section.substring(1);
            currentUid = section;
            break;
          case 2:
            queueIndex = section.toInt();
            break;
          case 3:
            isLooping = section.charAt(0) == '1';
            break;
          case 4:
            isShuffle = section.charAt(0) == '1';
            break;
          case 5:
            angSpeed = section.toInt();
            Serial.println("Setting max ang speed to:");
            Serial.println(angSpeed);
            angleStepper.setSpeed(angSpeed);
            break;
          
          case 6:
          Serial.println("Setting max dist speed to:");
            Serial.println(distSpeed);
            distSpeed = section.toInt();
            distStepper.setSpeed(distSpeed);
            break;
          case 7:
            hasX = true;
            tempX = section.toDouble();
            break;
          case 8:
            if (hasX) {
              remotePoint1.update(tempX, section.toDouble());
              remotePoint2.update(PolarPoint(angleStepper.currentPosition()/stepsPerRotation, distStepper.currentPosition()/axisSteps));
              lineProgress = 0;
            }
            break;
        }
      }
      i++;
      data = data.substring(index+1);
      index = data.indexOf(0x1D);
    }

    updateStatus();

    // int seperator1 = data.indexOf(0x1D);
    // int seperator2 = data.indexOf(0x1D, seperator1+1);
    // int seperator3 = data.indexOf(0x1D, seperator1+1);
    // int seperator4 = data.indexOf(0x1D, seperator1+1);
   


    // if (seperator1!=-1) {
    //   String section = data.substring(0,seperator1);
    //   if (seperator2==-1) {
    //     String lastSection = data.substring(seperator1+1);
    //   }
    // }
    // if (seperator2!=-1) {
    //   String section = data.substring(seperator1+1,seperator2);
    //   if (seperator3==-1) {
    //     String lastSection = data.substring(seperator2+1);
    //   }
    // }
    // if (seperator3!=-1) {
    //   String section = data.substring(seperator2+1,seperator3);
    //   if (seperator4==-1) {
    //     String lastSection = data.substring(seperator3+1);
    //   }
    // }
    // if (seperator4!=-1) {
    //   String section = data.substring(seperator3+1,seperator4);
    //   String lastSection = data.substring(seperator1+1);
    // }


    // Serial.print("firstSeperator: ");
    // Serial.println(firstSeperator);
    // // Broken input
    // if (length == 0) {
    //   return;
    // }
    // // Only pause argument
    // if (firstSeperator == -1 || length <= firstSeperator+1) { 
    //   Serial.println("Case 1");
    //   isPlaying = (data[0] == '1');
    //   return;
    // } 
    // if (oldData.length() > 1 && length > 1) {
    //   if (data.substr(1).compare(oldData.substr(1)) == 0) {
    //     isPlaying = (data[0] == '1');
    //     return;
    //   }
    // }
    // oldData = data;
    // int secondSeperator = data.find(0x1D, firstSeperator+1);
    //  Serial.print("secondSeperator: ");
    // Serial.println(secondSeperator);
    // isTrackSelected = (data[firstSeperator+1] == 'T');
    // //No queue index and is track
    // if ((secondSeperator < firstSeperator + 1 || length <= secondSeperator+1) && isTrackSelected) {
    //   Serial.println("Case 2");
    //   isPlaying = (data[0] == '1');
    //   std::__cxx11::string uid = data.substr(firstSeperator+2);
    //   currentTrack.close();
    //   queueIndex = -1;
    //   pendingTrack = String(uid.c_str());
    //   return;
    // } 
    // //Queue index present and is queue
    // else if (!isTrackSelected) {
    //   Serial.println("Case 3");
    //   String recievedQueueIndex = String(data.substr(secondSeperator+1).c_str());
    //   std::__cxx11::string queueUid = data.substr(firstSeperator+2, secondSeperator - (firstSeperator+2));
    //   isPlaying = (data[0] == '1');
    //   queueIndex = recievedQueueIndex.toInt();
    //   pendingQueue = queueUid.c_str();
    //   queueIndex = recievedQueueIndex.toInt();
    //   return;
    // } 
    // //Cant be no queue index and queue selected OR queue index but track selected
    // else {
    //   Serial.println("Case 4");
    //   isPlaying = false;
    //   isTrackSelected = true;
    //   queueIndex = -1;
    //   currentTrack.close();
    //   return;
    // }
	}

};

void setup() {
			// Initialize hardware serial for debugging
  		Serial.begin(115200);
      delay(1000);
    	// Initialize software serial for UART angle motor control
      Serial1.begin(115200);
      delay(1000);
      // Initialize software serial for UART distance motor control
      Serial2.begin(115200, SERIAL_8N1, 12, 22);

      delay(1000);

			// Configure Accelstepper
      angleStepper.setAcceleration(900);
      angleStepper.setSpeed(angSpeed);
      angleStepper.setMaxSpeed(angSpeed);
      distStepper.setAcceleration(900);
      distStepper.setSpeed(distSpeed);
      distStepper.setMaxSpeed(distSpeed);
      distStepper.setPinsInverted(true);

      pinMode(ANG_EN_PIN, OUTPUT);       // Set pinmodes
      pinMode(ANG_STEP_PIN, OUTPUT);
      pinMode(ANG_DIR_PIN, OUTPUT);

      pinMode(DIST_EN_PIN, OUTPUT);          
      pinMode(DIST_STEP_PIN, OUTPUT);
      pinMode(DIST_DIR_PIN, OUTPUT);

      pinMode(limitSwitchPin, INPUT_PULLUP);

      digitalWrite(ANG_EN_PIN, LOW);     // Enable TMC2209 boards
      digitalWrite(DIST_EN_PIN, LOW);

  ////////=====ANGLE UART CONFIGURATION====////////

      angleTMCDriver.begin();                // Start UART communication
      angleTMCDriver.pdn_disable(false);     // Use PDN/UART pin for communication
      angleTMCDriver.mstep_reg_select(1);    // Necessary for TMC2209 to set microstep register with UART
      angleTMCDriver.toff(5);                // Enables driver in software
      angleTMCDriver.rms_current(1000);      // Set motor RMS current in mA
      angleTMCDriver.microsteps(microstep);         // Set microsteps
      angleTMCDriver.en_spreadCycle(false);  // false = StealthChop(quiter) / true = SpreadCycle(faster)
      angleTMCDriver.pwm_autoscale(true);    // Needed for stealthChop

  ////////=====DISTANCE UART CONFIGURATION====////////

      distTMCDriver.begin();                // Start UART communication
      distTMCDriver.pdn_disable(false);     // Use PDN/UART pin for communication
      distTMCDriver.mstep_reg_select(1);    // Necessary for TMC2209 to set microstep register with UART
      distTMCDriver.toff(5);                // Enables driver in software
      distTMCDriver.rms_current(600);      // Set motor RMS current in mA
      distTMCDriver.microsteps(microstep);        // Set microsteps
      distTMCDriver.en_spreadCycle(false);  // false = StealthChop(quiter) / true = SpreadCycle(faster)
      distTMCDriver.pwm_autoscale(true);    // Needed for stealthChop

  steppers.addStepper(angleStepper);
  steppers.addStepper(distStepper);
  
  delay(1000);

  bool isHomed = false;
  int switchCount = 0;

  digitalWrite(DIST_DIR_PIN, LOW);
  while(isHomed == false) {
    if (digitalRead(limitSwitchPin) == 0) {
      digitalWrite(DIST_STEP_PIN, HIGH);
      delayMicroseconds((100*128)/microstep);
      digitalWrite(DIST_STEP_PIN, LOW);
      delayMicroseconds((100*128)/microstep);
      switchCount = 0;
    } else {
      switchCount++;
      if (switchCount >= 15) {
        isHomed = true;
      }
    }
  }

  isHomed = false;
  switchCount = 0;
  Serial.println("Homing...");
  digitalWrite(DIST_DIR_PIN, HIGH);
  while(isHomed == false) {
    if (digitalRead(limitSwitchPin) == 1) {
      digitalWrite(DIST_STEP_PIN, HIGH);
      delayMicroseconds((100*128)/microstep);
      digitalWrite(DIST_STEP_PIN, LOW);
      delayMicroseconds((100*128)/microstep);
      switchCount = 0;
    } else {
      switchCount++;
      if (switchCount >= 15) {
        Serial.println("Done homing!");
        isHomed = true;
      }
    }
  }
  distStepper.setCurrentPosition((microstep*200)*-0.45);
  distStepper.runToNewPosition(0);

Serial.begin(115200);
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }

  Serial.println("Starting BLE work!");

  NimBLEDevice::init("Tottori0001");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db, default +3db */
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);
  pWiiFiCredCharacteristic = pService->createCharacteristic(
    WIFI_CREDENTIAL_CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pWiiFiCredCharacteristic->setValue("");
  pWiiFiCredCharacteristic->setCallbacks(new WifiCredentialCallback());

  pStatusCharacteristic = pService->createCharacteristic(
    STATUS_CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ |
    NIMBLE_PROPERTY::NOTIFY
  );
  // pStatusCharacteristic->addDescriptor(new NimBLE2904());
  pStatusCharacteristic->setValue("0" + 0x1D + 0x1D + 0x1D + 0x1D);

  pRemoteCharacteristic = pService->createCharacteristic(
    REMOTE_CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::WRITE |
    NIMBLE_PROPERTY::NOTIFY
  );

  // pRemoteCharacteristic->addDescriptor(new NimBLE2904());
  pRemoteCharacteristic->setCallbacks(new RemoteCallback());
  pRemoteCharacteristic->setValue("0");

  pService->start();
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  advertisementData.setManufacturerData("Tottori");
  pAdvertising->setAdvertisementData(advertisementData);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  
  NimBLEDevice::startAdvertising();
  updateStatus();

  Serial.println("Characteristic defined! Now you can read it in your phone!");
  loadSettings();
  wifiAutoconnect();

  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);

  Serial.println("Initializing app...");

    Serial.println(esp_get_free_heap_size());


  ssl_client.setInsecure();

    Serial.println(esp_get_free_heap_size());


  initializeApp(aClient, app, getAuth(user_auth), aResult_no_callback);

    Serial.println(esp_get_free_heap_size());


  authHandler();

    Serial.println(esp_get_free_heap_size());


  app.getApp<Firestore::Documents>(Docs);

    Serial.println(esp_get_free_heap_size());


  // In case setting the external async result to the sync task (optional)
  // To unset, use unsetAsyncResult().
  //aClient.setAsyncResult(aResult_no_callback);

    Serial.println(esp_get_free_heap_size());


  app.getApp<Storage>(storage);

    Serial.println(esp_get_free_heap_size());
  updateStatus();
 
}

void loop() {
 
  app.loop();
  storage.loop();
  authHandler();
  Docs.loop();
  // Serial.println(esp_get_free_heap_size());

  if (WiFi.status() != WL_CONNECTED) {
    wifiAutoconnect(wifiPreference);
  } else if (app.ready()) {
    if (!pendingUid.isEmpty()) {
      if (isTrackSelected) {
        setCurrentTrack(pendingUid);
      } else {
        setCurrentQueue(pendingUid);
        setCurrentTrack(getTrackAtIndex(queueIndex));
      }
      pendingUid.clear();
    } else if (remotePoint2.exists) {
      if (!steppers.run()) {
        //TODO: change lineprogress++ to +=dist??
        lineProgress++;
        goalPoint = (remotePoint1.lerp(remotePoint2,lineProgress/subdivisions));
        if (goalPoint.distance > CENTER_DEADZONE) {
          moveMotorsTowards(goalPoint.getAngleMotorPos(), goalPoint.getDistanceMotorPos());
        }
      }
    } else if (isPlaying) {
      if (waitEnd != 0) {
        if (waitEnd > millis()) {
          delay(1);
        } else {
          waitEnd = 0;
        }
      } else if (currentTrack) {
        if (currentTrack.available()) {
          if (currentTrack.position() != trackPosition) {
            Serial.print("currentTrack.position(): ");
            Serial.println(currentTrack.position());
            Serial.print("trackPosition: ");
            Serial.println(trackPosition);
            currentTrack.seek(trackPosition);
          }
          if (lineProgress >= subdivisions) {
            lineProgress = 0;
            point1.x = point2.x;
            point1.y = point2.y;
            String line = currentTrack.readStringUntil('\n');
            trackPosition = currentTrack.position();
            // Serial.println(line);
            // Serial.print("currentTrack.position() : ");
            // Serial.print(currentTrack.position());
            // Serial.print("currentTrack.size() : ");
            // Serial.println(currentTrack.size());
            if (line.charAt(0) == 'M') {
              point2.x = 2*line.substring(1,line.indexOf(' ')).toDouble()-1;
              point2.y = 2*line.substring(line.indexOf(' ')+1).toDouble()-1;
              trackProgress += point1.distanceTo(point2)/2;
              updateStatus();
            }
          } else {
            if (!steppers.run()) {
              lineProgress++;
              goalPoint = (point1.lerp(point2,lineProgress/subdivisions));
              if (goalPoint.distance > CENTER_DEADZONE) {
                moveMotorsTowards(goalPoint.getAngleMotorPos(), goalPoint.getDistanceMotorPos());
              }
            }
          }
        }
        if (currentTrack.position() == currentTrack.size()) {
          Serial.println("Done with:");
          Serial.println(currentTrack.name());
          Serial.print("    queueIndex: ");
          Serial.println(queueIndex);
          next();
          Serial.println("Moving to:");
          Serial.println(currentTrack.name());            
          Serial.print("    queueIndex: ");
          Serial.println(queueIndex);
          delay(10000);
        }
      }
    }
  }
}

void authHandler()
{
    // Blocking authentication handler with timeout
    unsigned long ms = millis();
    while (app.isInitialized() && !app.ready() && millis() - ms < 120 * 1000)
    {
        // This JWT token process required for ServiceAuth and CustomAuth authentications
        JWT.loop(app.getAuth());
        printResult(aResult_no_callback);
    }
}

void printResult(AsyncResult &aResult)
{
    if (aResult.isEvent())
    {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.appEvent().message().c_str(), aResult.appEvent().code());
    }

    if (aResult.isDebug())
    {
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError())
    {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }

    if (aResult.available())
    {
        Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
    }
}

void printError(int code, const String &msg)
{
    Firebase.printf("Error, msg: %s, code: %d\n", msg.c_str(), code);
}

void fileCallback(File &file, const char *filename, file_operating_mode mode)
{
    // FILE_OPEN_MODE_READ, FILE_OPEN_MODE_WRITE and FILE_OPEN_MODE_APPEND are defined in this library
    // SD is defined in this example
    switch (mode)
    {
    case file_mode_open_read:
        file = SD.open(filename, FILE_OPEN_MODE_READ);
        break;
    case file_mode_open_write:
        file = SD.open(filename, FILE_OPEN_MODE_WRITE);
        break;
    case file_mode_open_append:
        file = SD.open(filename, FILE_OPEN_MODE_APPEND);
        break;
    case file_mode_remove:
        SD.remove(filename);
        break;
    default:
        break;
    }
}