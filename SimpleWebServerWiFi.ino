#include <SPI.h>
#include <WiFiS3.h> 
#include <ArduinoHttpClient.h> 
#include <DHT.h>
#include "arduino_secrets.h" 
#include <math.h>

// --- 1. 통신 및 센서 정의 ---
#define WIFI_SSID SECRET_SSID
#define WIFI_PASSWORD SECRET_PASS
#define NESTJS_SERVER_IP SECRET_IP 
const int NESTJS_SERVER_PORT = 3000;
const char NESTJS_ENDPOINT[] = "/receive-data"; // NestJS 서버 엔드포인트

const int DHT_DATA_PIN = 2;
const int GAS_SENSOR_PIN = A0;
const int DHT_TYPE = DHT11;

// 라이브러리 인스턴스
WiFiClient wifi;
HttpClient client = HttpClient(wifi, NESTJS_SERVER_IP, NESTJS_SERVER_PORT);
DHT dhtSensor(DHT_DATA_PIN, DHT_TYPE);

// --- 2. EKF 및 센서 변수 ---
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

// ** 예측 데이터 변수 (전송을 위해 전역 변수로 선언) **
float predictedHI = 0.0; // 이제 "10초간의 체감 온도 변화량(Delta)"을 의미
float predictedG = 0.0;  // 이제 "10초간의 가스 농도 변화량(Delta)"을 의미

// --- 3. 위험도 평가 (Risk Assessment) 변수 및 함수 ---
const int RISK_HISTORY_SIZE = 10; // 과거 10초간의 데이터를 사용 (100 -> 10으로 복원하여 메모리 안정화)
float HIHistory[RISK_HISTORY_SIZE] = {0.0}; // 체감 온도 히스토리
float GHistory[RISK_HISTORY_SIZE] = {0.0};  // 가스(PPM) 히스토리
int HistoryIndex = 0; // 현재 채워진 데이터 수

// ** 패턴 분석 상수 (최적화된 동적 임계값 로직) **
// 예측 안전 계수: 이 계수는 이제 사용하지 않으며, 값 1.0으로 유지합니다.
const float HI_PREDICTIVE_SAFETY_FACTOR = 1.0; 
const float G_PREDICTIVE_SAFETY_FACTOR = 1.0; 

// ** 추세 위험 (Trend Risk) 판단을 위한 최대 증가량 임계값 **
// 이 값보다 10초간 증가량이 클 경우에만 추세 위험으로 판단합니다.
const float HI_MAX_INCREASE_DELTA = 0.5;   // 10초간 0.5C 이상 증가 시 위험
const float G_MAX_INCREASE_DELTA = 50.0;  // 10초간 50 PPM 이상 증가 시 위험

// ** ADAPTIVE THRESHOLD VARIABLES (자동 보정 로직) **
float LTSD_HI = 0.0; // Long-Term Standard Deviation for HI (EMA 기반)
float LTSD_G = 0.0;  // Long-Term Standard Deviation for Gas (EMA 기반)
const float SD_SMOOTHING_FACTOR = 0.05; // 0.05로 낮춰 적응 속도를 늦춥니다 (덜 민감하게).
const float SD_SAFETY_MULTIPLIER = 5.0; // 안전 마진: 평소 변동폭의 5배를 넘으면 경고 (패턴 위험 완화)

// 불안정성(표준 편차) 임계값: 패턴 불안정성을 감지하기 위한 보조 지표 (RUNTIME ADJUSTED)
float HI_SD_THRESHOLD = 0.0; // 초기값: 첫 SD 계산 후 자동 보정됨
float G_SD_THRESHOLD = 0.0; // 초기값: 첫 SD 계산 후 자동 보정됨

// ** 스파이크 위험 (Spike Risk) 감지 임계값 (EKF Innovation Check) **
// EKF 안정 상태 대비 Raw 값이 이 임계값 이상 높을 경우 즉각적인 '스파이크 위험'으로 간주
const float HI_INNOVATION_THRESHOLD = 1.0;   // Raw HI가 Filtered HI보다 1.0C 초과 시
const float G_INNOVATION_THRESHOLD = 100.0; // Raw Gas가 Filtered Gas보다 100 PPM 초과 시

String intensityMessage = "DATA INIT"; // 초기 상태 메시지

// --- 상수 설정 ---
// UNO R4 Minima는 12-bit ADC (0 ~ 4095)
float filtered = 0.0f;

const int   ADC_MAX        = 4095;
const int   PULSE_SENSOR_PIN = A1;

// 필터링 및 샘플링 주파수
const float FS             = 100.0f; // 샘플링 주파수 (Hz)
const float DT             = 1.0f / FS; // 샘플링 주기 (초)
const float FC_HP          = 0.7f; // HPF 컷오프 주파수 (Hz)
const float FC_LP          = 5.0f; // LPF 컷오프 주파수 (Hz)

// 시리얼 플로터 출력 스케일 설정
// UNO R4 (12-bit)는 UNO R3 (10-bit)보다 데이터 범위가 4배 넓으므로,
// 출력 값의 크기를 적절히 조절하기 위한 게인입니다.
// (12-bit/10-bit 비율 4에 맞추어 6.0f로 설정되어 있음)
const float SCALING_GAIN   = 6.0f;
const float DC_OFFSET      = 512.0f; // 시리얼 플로터 중앙값 (0~1000 스케일 기준)
const int   PLOT_MAX       = 1000;

// --- 필터 계수 및 상태 변수 ---
float aHP, bLP; // HPF 및 LPF 필터 계수
float x_prev=0.0f;   // 이전 입력 값 (rawADC)
float yHP_prev=0.0f; // 이전 HPF 출력 값
float yLP_prev=0.0f; // 이전 LPF/BPF 출력 값

float analogToPPM(int analogValue) {
    return map(analogValue, 0, 1023, 0, (int)GAS_MAX_PPM);
}

// 확장 칼만 필터 실행 함수
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

// 표준 편차 계산 함수 (데이터의 불안정성 패턴 분석)
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

// 평균 계산 함수
float calculateMean(float data[], int size) {
    if (size == 0) return 0.0;
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

// 히스토리 업데이트 및 단계적 초기화 함수 (요청 반영)
void updateHistory() {
    float hiValue = filteredHI;
    float gValue = filteredG;

    // 1. 단계적 초기화 (HistoryIndex < RISK_HISTORY_SIZE일 때)
    if (HistoryIndex < RISK_HISTORY_SIZE) {
        
        HIHistory[HistoryIndex] = hiValue;
        GHistory[HistoryIndex] = gValue;

        // HistoryIndex보다 큰 인덱스는 현재 값으로 모두 채움 (사용자 요청 반영)
        for (int i = HistoryIndex + 1; i < RISK_HISTORY_SIZE; i++) {
            HIHistory[i] = hiValue;
            GHistory[i] = gValue;
        }

        HistoryIndex++;
    } else {
        // 2. 실시간 업데이트 (슬라이딩 윈도우)
        
        // 10개가 모두 채워진 후에는 0 <- 1, 1 <- 2 ... 8 <- 9로 쉬프트
        for (int i = 0; i < RISK_HISTORY_SIZE - 1; i++) {
            HIHistory[i] = HIHistory[i + 1];
            GHistory[i] = GHistory[i + 1];
        }
        // 가장 최근 데이터 삽입 (인덱스 RISK_HISTORY_SIZE - 1)
        HIHistory[RISK_HISTORY_SIZE - 1] = filteredHI;
        GHistory[RISK_HISTORY_SIZE - 1] = filteredG;
    }
}

// ** 새로운 함수: 표준 편차 임계값을 자동으로 보정합니다 (EMA 기반) **
void updateAdaptiveThresholds(float currentSDHI, float currentSDG) {
    
    // Initializing flag check: if LTSD is near zero, it's the first run with valid data
    bool isInitialRun = (LTSD_HI < 0.001 && LTSD_G < 0.001); 

    // 1. Long-Term Standard Deviation (LTSD) 계산 (EMA 적용)
    if (isInitialRun) {
        LTSD_HI = currentSDHI; 
        LTSD_G = currentSDG;   
    } else {
        LTSD_HI = (SD_SMOOTHING_FACTOR * currentSDHI) + (1.0 - SD_SMOOTHING_FACTOR) * LTSD_HI;
        LTSD_G = (SD_SMOOTHING_FACTOR * currentSDG) + (1.0 - SD_SMOOTHING_FACTOR) * LTSD_G;
    }

    // 2. 임계값 설정
    HI_SD_THRESHOLD = LTSD_HI * SD_SAFETY_MULTIPLIER;
    G_SD_THRESHOLD = LTSD_G * SD_SAFETY_MULTIPLIER;
    
    // 3. 최소 임계값 보장 
    if (HI_SD_THRESHOLD < 0.1) HI_SD_THRESHOLD = 0.1; 
    if (G_SD_THRESHOLD < 1.0) G_SD_THRESHOLD = 1.0; 
}


// 위험도 평가 및 메시지 설정 함수
void assessRiskAndSetIntensity() {
    // Determine the current size of the history to use for calculation
    int currentSize = (HistoryIndex == 0) ? 0 : RISK_HISTORY_SIZE;
    if (HistoryIndex < RISK_HISTORY_SIZE) {
        currentSize = HistoryIndex;
    }
    
    // Safety check: ensure at least RISK_HISTORY_SIZE points are collected before assessing risk
    // 변경: 전체 10개 데이터가 채워져야 위험 분석을 시작합니다.
    if (currentSize < RISK_HISTORY_SIZE) { 
        intensityMessage = "DATA INIT"; 
        return;
    }

    // 1. 10초간의 변화 추세 (Delta) 계산
    float deltaHI = HIHistory[currentSize - 1] - HIHistory[0];
    float deltaG = GHistory[currentSize - 1] - GHistory[0];

    // 2. 10초간의 불안정성 패턴 (Standard Deviation) 계산
    float sdHI = calculateStandardDeviation(HIHistory, currentSize);
    float sdG = calculateStandardDeviation(GHistory, currentSize);
    
    // ** 2.5. ADAPTIVE THRESHOLD UPDATE (자동 보정) **
    updateAdaptiveThresholds(sdHI, sdG); 
    
    // 3. 예측 값 전역 변수 업데이트: 이제 델타(변화량) 자체를 전송합니다.
    predictedHI = deltaHI;
    predictedG = deltaG;

    // 4. 위험도 판단 (총 3가지 기준 사용)
    
    // 4.1. 추세 위험 (Trend Risk): 델타가 설정된 최대 증가량을 초과하는 경우만 위험으로 간주 (감소 추세 시 경고 방지)
    bool hiTrendRisk = (deltaHI > 0) && (deltaHI > HI_MAX_INCREASE_DELTA); 
    bool gasTrendRisk = (deltaG > 0) && (deltaG > G_MAX_INCREASE_DELTA); 

    // 4.2. 패턴 위험 (Pattern Risk): SD가 자동 보정된 임계값을 초과
    bool hiPatternRisk = sdHI > HI_SD_THRESHOLD; 
    bool gasPatternRisk = sdG > G_SD_THRESHOLD; 
    
    // 4.3. 스파이크 위험 (Spike Risk): Raw 값이 EKF 안정 상태를 급격히 벗어남
    float rawGasPPM = analogToPPM(gasRaw);
    float rawHI = dhtSensor.computeHeatIndex(temperatureRaw, humidityRaw, false);
    
    bool hiSpikeRisk = rawHI > (filteredHI + HI_INNOVATION_THRESHOLD); 
    bool gasSpikeRisk = rawGasPPM > (filteredG + G_INNOVATION_THRESHOLD); 
    
    // 최종 위험 통합 (OR 조건)
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
    
    // 디버깅 출력 (패턴 분석 결과 포함)
    Serial.print("PATTERN CHECK (Size:"); Serial.print(currentSize);
    // 현재 필터링된 값
    Serial.print("): F_HI="); Serial.print(filteredHI, 2);
    // 10초간의 변화량 (Delta) 출력
    Serial.print(", Delta_HI="); Serial.print(deltaHI, 3); 
    Serial.print(" (SD="); Serial.print(sdHI, 3);
    // Adaptive SD Threshold 출력
    Serial.print(", ADAPTIVE_SD_T="); Serial.print(HI_SD_THRESHOLD, 3); 
    
    Serial.print("), F_G="); Serial.print(filteredG, 0);
    // 10초간의 변화량 (Delta) 출력
    Serial.print(", Delta_G="); Serial.print(deltaG, 1); 
    Serial.print(" (SD="); Serial.print(sdG, 1);
    // Adaptive SD Threshold 출력
    Serial.print(", ADAPTIVE_SD_T="); Serial.print(G_SD_THRESHOLD, 1); 
    Serial.print(") -> Message: "); Serial.println(intensityMessage);
}

// --- 4. 통신 함수 ---

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
    // Nest.js DTO 형식에 맞춰 JSON Payload 생성
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

// --- 5. Arduino Setup & Loop ---

void setup() {
    Serial.begin(9600);

    // HPF (RC high-pass filter) 계수 계산
    // y[n] = a * (y[n-1] + x[n] - x[n-1])
    float RC_HP = 1.0f / (2.0f * M_PI * FC_HP); // M_PI는 math.h의 파이 상수
    aHP = RC_HP / (RC_HP + DT);

    // LPF (RC low-pass filter) 계수 계산
    // y[n] = y[n-1] + b * (x[n] - y[n-1])
    float RC_LP = 1.0f / (2.0f * M_PI * FC_LP);
    bLP = DT / (RC_LP + DT);

    dhtSensor.begin();
    pinMode(GAS_SENSOR_PIN, INPUT);
    pinMode(13, OUTPUT); // 예: 빨간색 LED (HIGH 위험)
    pinMode(12, OUTPUT); // 예: 노란색 LED (중간 위험)
    connectWiFi();
}

void loop() {

    // 1. 센서 데이터 읽기
    // 아날로그 데이터는 0~4095 (UNO R4 Minima 기준)
    int rawADC = analogRead(PULSE_SENSOR_PIN);
    float x    = (float)rawADC;

    // 2. 밴드패스 필터링 (HPF -> LPF)

    // 2-1. 1차 HPF 적용 (DC 성분 제거)
    // HPF: aHP * (yHP_prev + x - x_prev)
    float yHP = aHP * (yHP_prev + x - x_prev);

    // 2-2. 1차 LPF 적용 (노이즈 및 고주파 성분 제거)
    // BPF (yLP) = LPF(yHP)
    // LPF: yLP_prev + bLP * (yHP - yLP_prev)
    float yBP = yLP_prev + bLP * (yHP - yLP_prev);

    // 3. 필터 상태 업데이트
    x_prev   = x;
    yHP_prev = yHP;
    yLP_prev = yBP;

    // 4. 시리얼 플로터 출력을 위한 스케일링 및 클리핑
    // 필터링된 신호는 DC 성분이 제거되어 0 근처에서 진동합니다.
    // 시리얼 플로터에 잘 보이도록 게인을 곱하고 오프셋을 더해 0~1000 범위로 스케일링합니다.
    filtered = (yBP * SCALING_GAIN) + DC_OFFSET;

    // 클리핑: 값이 0~1000 범위를 벗어나지 않도록 보정
    if (filtered < 0.0f) filtered = 0.0f;
    if (filtered > (float)PLOT_MAX) filtered = (float)PLOT_MAX;

    // 1. Raw 센서 데이터 읽기
    humidityRaw = dhtSensor.readHumidity();
    temperatureRaw = dhtSensor.readTemperature();
    gasRaw = analogRead(GAS_SENSOR_PIN);
    
    // 디버깅을 위한 Raw 데이터 로깅
    Serial.print("RAW DATA: T="); Serial.print(temperatureRaw, 1); 
    Serial.print("C, H="); Serial.print(humidityRaw, 0); 
    Serial.print("%, G(Raw)="); Serial.println(gasRaw);
    
    if (isnan(temperatureRaw) || isnan(humidityRaw)) {
        Serial.println(F("DHT Read Failed. Skipping EKF cycle."));
        delay(1000); 
        return; 
    }
    
    // 2. EKF 필터링 실행
    runEKF();

    // 3. 히스토리 업데이트 및 위험도 평가
    updateHistory();
    assessRiskAndSetIntensity();

    // 디버깅을 위한 EKF 데이터 로깅 (체감 온도 및 현재 상태 포함)
    Serial.print("EKF DATA: T="); Serial.print(filteredT, 2); 
    Serial.print("C, H="); Serial.print(filteredH, 2); 
    Serial.print("%, G(PPM)="); Serial.print(filteredG, 0);
    Serial.print(" | HI="); Serial.print(filteredHI, 2); 
    Serial.print(" | INTENSITY="); Serial.println(intensityMessage);
    
    // 4. 1초마다 데이터 전송 시도
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();
        sendDataToServer();
    }

// --- LED 제어 로직 시작 ---
    if (intensityMessage == "TAKE A BREAK") {
        // 복합 위험 (가장 높음): 빨간색 LED 깜빡임 (예시)
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (intensityMessage == "MANAGE HEAT" || intensityMessage == "MANAGE BREATH") {
        // 호흡 위험: 노란색 LED 켜짐 (MANAGE HEAT와 동일하게 처리)
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    } else {
        // OPTIMAL 또는 DATA INIT: 모든 LED 끔
        digitalWrite(13, LOW);
        digitalWrite(12, LOW);
    }
    
    delay(1000); 
}
