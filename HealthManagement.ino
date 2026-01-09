#include <SPI.h>
#include <WiFiS3.h> 
#include <ArduinoHttpClient.h> 
#include <DHT.h>
#include "arduino_secrets.h" 
#include <math.h>

#define WIFI_SSID SECRET_SSID
#define WIFI_PASSWORD SECRET_PASS
#define NESTJS_SERVER_IP SECRET_IP 
#define NESTJS_SERVER_PORT SECRET_PORT
#define NESTJS_ENDPOINT[] SECRET_ENDPOINT
#define DHT_DATA_PIN SECRET_DHT
#define GAS_SENSOR_PIN SECRET_GAS
#define PULSE_SENSOR_PIN SECRET_PULSE

const char NESTJS_ENDPOINT[] = "/receive-data"; 

const int DHT_DATA_PIN = 2;
const int DHT_TYPE = DHT11;

const int GAS_SENSOR_PIN = A0;
const int PULSE_SENSOR_PIN = A1;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, NESTJS_SERVER_IP, NESTJS_SERVER_PORT);
DHT dhtSensor(DHT_DATA_PIN, DHT_TYPE);

float temperatureRaw = 0.0;
float humidityRaw = 0.0;
int gasRaw = 0; 

float filteredT = 0.0; 
float filteredH = 0.0; 
float filteredG = 0.0; 
float filteredHI = 0.0; 

float EKF_state[3] = {0.0, 0.0, 0.0}; 
float EKF_P[3] = {1.0, 1.0, 100.0}; 
const float PROCESS_NOISE_Q[3] = {0.01, 0.01, 1.0};      
const float MEASUREMENT_NOISE_R[3] = {0.1, 0.1, 10.0};   
const float GAS_MAX_PPM = 5000.0; 

float predictedHI = 0.0; 
float predictedG = 0.0;  

const int RISK_HISTORY_SIZE = 10; 
float HIHistory[RISK_HISTORY_SIZE] = {0.0}; 
float GHistory[RISK_HISTORY_SIZE] = {0.0};  
int HistoryIndex = 0; 

const float HI_PREDICTIVE_SAFETY_FACTOR = 1.0; 
const float G_PREDICTIVE_SAFETY_FACTOR = 1.0; 

const float HI_MAX_INCREASE_DELTA = 0.5;   
const float G_MAX_INCREASE_DELTA = 50.0;  

float LTSD_HI = 0.0; 
float LTSD_G = 0.0;  
const float SD_SMOOTHING_FACTOR = 0.05;
const float SD_SAFETY_MULTIPLIER = 5.0; 

float HI_SD_THRESHOLD = 0.0; 
float G_SD_THRESHOLD = 0.0; 

const float HI_INNOVATION_THRESHOLD = 1.0;   
const float G_INNOVATION_THRESHOLD = 100.0; 

String intensityMessage = "DATA INIT"; 

float filtered = 0.0f;

const int   ADC_MAX        = 4095;
const float FS             = 100.0f; 
const float DT             = 1.0f / FS; 
const float FC_HP          = 0.7f; 
const float FC_LP          = 5.0f; 

const float SCALING_GAIN   = 6.0f;
const float DC_OFFSET      = 512.0f; 
const int   PLOT_MAX       = 1000;

float aHP, bLP; 
float x_prev=0.0f;  
float yHP_prev=0.0f; 
float yLP_prev=0.0f; 

float analogToPPM(int analogValue) {
    return map(analogValue, 0, 1023, 0, (int)GAS_MAX_PPM);
}

void runEKF() {
    float rawGasPPM = analogToPPM(gasRaw);
    float measurement[3] = {temperatureRaw, humidityRaw, rawGasPPM};
    
    if (EKF_state[0] == 0.0 && !isnan(temperatureRaw)) {
        EKF_state[0] = temperatureRaw;
        EKF_state[1] = humidityRaw;
        EKF_state[2] = rawGasPPM;
    }

    for (int i = 0; i < 3; i++) {
        EKF_P[i] += PROCESS_NOISE_Q[i];
    }

    float K[3]; 
    for (int i = 0; i < 3; i++) {
        K[i] = EKF_P[i] / (EKF_P[i] + MEASUREMENT_NOISE_R[i]);
        float innovation = measurement[i] - EKF_state[i];
        EKF_state[i] += K[i] * innovation;
        EKF_P[i] = (1.0 - K[i]) * EKF_P[i];
    }

    filteredT = EKF_state[0];
    filteredH = EKF_state[1];
    filteredG = EKF_state[2];
    
    if (!isnan(filteredT) && !isnan(filteredH)) {
        filteredHI = dhtSensor.computeHeatIndex(filteredT, filteredH, false); 
    }
}

float calculateStandardDeviation(float data[], int size) {
    if (size == 0) return 0.0;
    
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    float mean = sum / size;
    
    float variance = 0.0;
    for (int i = 0; i < size; i++) {
        variance += pow(data[i] - mean, 2);
    }
    
    return sqrt(variance / size);
}

float calculateMean(float data[], int size) {
    if (size == 0) return 0.0;
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

void updateHistory() {
    float hiValue = filteredHI;
    float gValue = filteredG;
    if (HistoryIndex < RISK_HISTORY_SIZE) {
        
        HIHistory[HistoryIndex] = hiValue;
        GHistory[HistoryIndex] = gValue;

        for (int i = HistoryIndex + 1; i < RISK_HISTORY_SIZE; i++) {
            HIHistory[i] = hiValue;
            GHistory[i] = gValue;
        }

        HistoryIndex++;
    } else {
        for (int i = 0; i < RISK_HISTORY_SIZE - 1; i++) {
            HIHistory[i] = HIHistory[i + 1];
            GHistory[i] = GHistory[i + 1];
        }
        HIHistory[RISK_HISTORY_SIZE - 1] = filteredHI;
        GHistory[RISK_HISTORY_SIZE - 1] = filteredG;
    }
}

void updateAdaptiveThresholds(float currentSDHI, float currentSDG) {
    bool isInitialRun = (LTSD_HI < 0.001 && LTSD_G < 0.001); 

    if (isInitialRun) {
        LTSD_HI = currentSDHI; 
        LTSD_G = currentSDG;   
    } else {
        LTSD_HI = (SD_SMOOTHING_FACTOR * currentSDHI) + (1.0 - SD_SMOOTHING_FACTOR) * LTSD_HI;
        LTSD_G = (SD_SMOOTHING_FACTOR * currentSDG) + (1.0 - SD_SMOOTHING_FACTOR) * LTSD_G;
    }

    HI_SD_THRESHOLD = LTSD_HI * SD_SAFETY_MULTIPLIER;
    G_SD_THRESHOLD = LTSD_G * SD_SAFETY_MULTIPLIER;
    
    if (HI_SD_THRESHOLD < 0.1) HI_SD_THRESHOLD = 0.1; 
    if (G_SD_THRESHOLD < 1.0) G_SD_THRESHOLD = 1.0; 
}


void assessRiskAndSetIntensity() {
    int currentSize = (HistoryIndex == 0) ? 0 : RISK_HISTORY_SIZE;
    if (HistoryIndex < RISK_HISTORY_SIZE) {
        currentSize = HistoryIndex;
    }
    
    if (currentSize < RISK_HISTORY_SIZE) { 
        intensityMessage = "DATA INIT"; 
        return;
    }

    float deltaHI = HIHistory[currentSize - 1] - HIHistory[0];
    float deltaG = GHistory[currentSize - 1] - GHistory[0];

    float sdHI = calculateStandardDeviation(HIHistory, currentSize);
    float sdG = calculateStandardDeviation(GHistory, currentSize);
    
    updateAdaptiveThresholds(sdHI, sdG); 
    
    predictedHI = deltaHI;
    predictedG = deltaG;

    bool hiTrendRisk = (deltaHI > 0) && (deltaHI > HI_MAX_INCREASE_DELTA); 
    bool gasTrendRisk = (deltaG > 0) && (deltaG > G_MAX_INCREASE_DELTA); 
    bool hiPatternRisk = sdHI > HI_SD_THRESHOLD; 
    bool gasPatternRisk = sdG > G_SD_THRESHOLD; 
    
    float rawGasPPM = analogToPPM(gasRaw);
    float rawHI = dhtSensor.computeHeatIndex(temperatureRaw, humidityRaw, false);
    
    bool hiSpikeRisk = rawHI > (filteredHI + HI_INNOVATION_THRESHOLD); 
    bool gasSpikeRisk = rawGasPPM > (filteredG + G_INNOVATION_THRESHOLD); 
    bool hiRisk = hiTrendRisk || hiPatternRisk || hiSpikeRisk;
    bool gasRisk = gasTrendRisk || gasPatternRisk || gasSpikeRisk;

    if (hiRisk && gasRisk) {
        intensityMessage = "TAKE A BREAK";
    } else if (hiRisk) {
        intensityMessage = "MANAGE HEAT";
    } else if (gasRisk) {
        intensityMessage = "MANAGE BREATH";
    } else {
        intensityMessage = "OPTIMAL"; 
    }
    
    // Serial.print("PATTERN CHECK (Size:"); Serial.print(currentSize);
    // Serial.print("): F_HI="); Serial.print(filteredHI, 2);
    // Serial.print(", Delta_HI="); Serial.print(deltaHI, 3); 
    // Serial.print(" (SD="); Serial.print(sdHI, 3);
    // Serial.print(", ADAPTIVE_SD_T="); Serial.print(HI_SD_THRESHOLD, 3); 
    // Serial.print("), F_G="); Serial.print(filteredG, 0);
    // Serial.print(", Delta_G="); Serial.print(deltaG, 1); 
    // Serial.print(" (SD="); Serial.print(sdG, 1);
    // Serial.print(", ADAPTIVE_SD_T="); Serial.print(G_SD_THRESHOLD, 1); 
    // Serial.print(") -> Message: "); Serial.println(intensityMessage);
}

void connectWiFi() {
    Serial.print(F("Connecting to "));
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }

    Serial.println(F("\nWiFi Connected."));
    Serial.print(F("IP Address: "));
    Serial.println(WiFi.localIP());
}


void sendDataToServer() {
    String jsonPayload = "{\"temperature\": " + String(filteredT, 2) + 
                         ", \"humidity\": " + String(filteredH, 2) + 
                         ", \"heatIndex\": " + String(filteredHI, 2) +

                         ", \"gasValue\": " + String(filteredG, 0) + 

                         ", \"pulseValue\": " + String(filtered, 2) +

                         ", \"predictedHI\": " + String(predictedHI, 2) + 
                         ", \"predictedGas\": " + String(predictedG, 0) +
                         ", \"intensity\": \"" + intensityMessage + "\"" + 
                         "}"; 

    Serial.println(F("\n--- Sending POST Request ---"));
    Serial.print(F("Payload: "));
    client.beginRequest();
    client.post(NESTJS_ENDPOINT);
    
    client.sendHeader(F("Content-Type"), F("application/json"));
    client.sendHeader(F("Content-Length"), jsonPayload.length());
    
    client.endRequest();
    client.write((uint8_t*)jsonPayload.c_str(), jsonPayload.length());

    int statusCode = client.responseStatusCode();
    
    Serial.print(F("Status Code: "));
    Serial.println(statusCode);
    
    String responseBody = "";
    while (client.available()) {
        responseBody += (char)client.read();
    }

    if (statusCode == 200 || statusCode == 201) {
        Serial.println(F("Communication SUCCESS!"));
    } else {
        Serial.print(F("Communication FAILED. Response: "));
        Serial.println(responseBody);
    }
}

void setup() {
    Serial.begin(9600);
    float RC_HP = 1.0f / (2.0f * M_PI * FC_HP); 
    aHP = RC_HP / (RC_HP + DT);
    float RC_LP = 1.0f / (2.0f * M_PI * FC_LP);
    bLP = DT / (RC_LP + DT);

    dhtSensor.begin();
    pinMode(GAS_SENSOR_PIN, INPUT);
    pinMode(13, OUTPUT); 
    pinMode(12, OUTPUT); 
    connectWiFi();
}

void loop() {
    int rawADC = analogRead(PULSE_SENSOR_PIN);
    float x    = (float)rawADC;
    float yHP = aHP * (yHP_prev + x - x_prev);
    float yBP = yLP_prev + bLP * (yHP - yLP_prev);

    x_prev   = x;
    yHP_prev = yHP;
    yLP_prev = yBP;
    filtered = (yBP * SCALING_GAIN) + DC_OFFSET;

    if (filtered < 0.0f) filtered = 0.0f;
    if (filtered > (float)PLOT_MAX) filtered = (float)PLOT_MAX;

    humidityRaw = dhtSensor.readHumidity();
    temperatureRaw = dhtSensor.readTemperature();
    gasRaw = analogRead(GAS_SENSOR_PIN);
    
    Serial.print("RAW DATA: T="); Serial.print(temperatureRaw, 1); 
    Serial.print("C, H="); Serial.print(humidityRaw, 0); 
    Serial.print("%, G(Raw)="); Serial.println(gasRaw);
    
    if (isnan(temperatureRaw) || isnan(humidityRaw)) {
        Serial.println(F("DHT Read Failed. Skipping EKF cycle."));
        delay(1000); 
        return; 
    }
    
    runEKF();
    updateHistory();
    assessRiskAndSetIntensity();

    // Serial.print("EKF DATA: T="); Serial.print(filteredT, 2); 
    // Serial.print("C, H="); Serial.print(filteredH, 2); 
    // Serial.print("%, G(PPM)="); Serial.print(filteredG, 0);
    // Serial.print(" | HI="); Serial.print(filteredHI, 2); 
    // Serial.print(" | INTENSITY="); Serial.println(intensityMessage);
    
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();
        sendDataToServer();
    }

    if (intensityMessage == "TAKE A BREAK") {
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (intensityMessage == "MANAGE HEAT" || intensityMessage == "MANAGE BREATH") {
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    } else {
        digitalWrite(13, LOW);
        digitalWrite(12, LOW);
    }
    
    delay(1000); 
}
