#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Preferences.h>
#include <cmath>
#include <WebServer.h>
#include <Update.h>
#include "DisplayZh.h"
#include <esp_system.h>

// === 硬件配置 ===
#define ONE_WIRE_BUS 4    //主温控器数据引脚
#define CHAMBER_TEMP_PIN 16 // 仓温数据引脚（可选）
#define LED_PIN    48    //LED引脚
#define OLED_SDA_PIN 6   // 屏幕SDA引脚
#define OLED_SCL_PIN 5   // 屏幕SCL引脚
#define AHT30_SDA_PIN 9    // 温湿度计SDA引脚
#define AHT30_SCL_PIN 10   // 温湿度计SCL引脚
#define TEMP_BUTTON_PIN     17  // 温度控制按钮引脚
#define TIME_BUTTON_PIN     18  // 时间控制按钮引脚
#define MODE_BUTTON_PIN     8   // 模式控制按钮引脚
#define FACTORY_BUTTON_PIN  19  // 重置参数按钮引脚
#define HEATER_PWM_PIN   7  // PTC加热器PWM控制引脚
#define FAN_PWM_PIN    15  // 风扇PWM调速引脚
#define EXHAUST_INA_PIN 11  // 电机正反控制INA引脚
#define EXHAUST_INB_PIN 12  // 电机正反控制INB引脚

#define OLED_RESET    -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
#define LED_NUMPIXELS 1      //LED数量
Adafruit_NeoPixel pixels(LED_NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWireChamber(CHAMBER_TEMP_PIN);
DallasTemperature sensors(&oneWire);
DallasTemperature chamberSensor(&oneWireChamber);
Adafruit_AHTX0 aht;

// === 系统参数 ===
#define PWM_FREQUENCY 20000   // 20kHz PWM频率
#define PWM_RESOLUTION 8      // 8位分辨率 (0-255)
#define MAX_TEMP 100
#define MIN_TEMP 40
#define TEMP_HYSTERESIS 0.5   // 温度控制精度
#define SAFETY_OVERTEMP 10     // 超过设定温度触发保护
#define MIN_PWM 0            // 最小PWM值，维持加热器工作
#define HEATER_PWM_CHANNEL    0   //PTC加热器PWM通道
#define FAN_PWM_CHANNEL    1      // 风扇PWM通道
#define FAN_MIN_PWM        100     // 风扇最小安全功率,防止停转和过热

// 按键状态
bool lastTempButtonState = false;   // 低电平为按下
bool lastTimeButtonState = false;
bool lastModeButtonState = false;
bool lastFactoryButtonState = false;
bool pidHoldActive = false;
bool tempButtonLongPressed = false;
bool timeButtonLongPressed = false;
bool modeButtonLongPressed = false;
bool isMOdeButtonLongPressed = false;
bool factoryButtonLongPressed = false;
unsigned long tempButtonPressStart = 0;
unsigned long timeButtonPressStart = 0;
unsigned long modeButtonPressStart = 0;
unsigned long factoryButtonPressStart = 0;
unsigned long PIDPressStart = 0;
const unsigned long LONG_PRESS_DURATION = 500; // 长按时间阈值(ms)
const unsigned long LONG_PRESS_REPEAT = 500; // 长按连续触发间隔(ms)
const unsigned long DEBOUNCE_DELAY = 50; // 消抖时间(ms)
const unsigned long FACTORY_DELAY = 15000; // 恢复出厂设置长按时间(ms)

// PID自动整定参数
#define PID_AUTOTUNE_TARGET 65.0    // 自动整定目标温度默认值
#define PID_AUTOTUNE_NOISE_BAND 5.0 // 噪音带宽
#define PID_AUTOTUNE_OUTPUT_STEP 255.0
#define PID_AUTOTUNE_SAMPLE_TIME 1000 // 采样时间(ms)
#define PID_AUTOTUNE_MIN_CYCLES 3     // 最小震荡次数
#define PID_AUTOTUNE_MAX_CYCLES 6     // 最大震荡次数
#define PID_AUTO_TIME_OUT 18000000    //PID超时时间
#define MIN_AMPLITUDE 0.6             // 最小可接受振幅
#define MAX_AMPLITUDE 10.0            // 最大可接受振幅
#define STABILITY_THRESHOLD 0.3       // 周期稳定性阈值(30%)
#define TEMP_FILTER_WEIGHT 0.2        // 温度滤波权重

// PID稳定性相关参数
const double PID_SETPOINT_RAMP_RATE = 0.45;     // 设定值斜坡(℃/s)，避免一步到位导致积分冲过头
const double PID_I_ACTIVE_BAND = 6.0;           // 误差小于该值才放大全部积分
const double PID_I_REDUCED_FACTOR = 0.35;       // 误差大时减少积分的系数
const double PID_NEAR_SETPOINT_BAND = 2.5;      // 接近目标时收敛功率限制
const int PID_NEAR_SETPOINT_MAX_OUTPUT = 220;   // 接近目标时允许的最大PWM

// PID用-温度历史记录
const int HISTORY_SIZE = 180; // 历史数据
float tempHistory[HISTORY_SIZE];
int historyIndex = 0;

// === WiFi AP模式配置 ===
#define WIFI_TIMEOUT 10000    // WiFi连接超时时间(ms)
WebServer server(80);         // 创建Web服务器实例
bool apMode = false;          // 默认关闭
String apSSID = "ams_lite_dryer";  // AP热点名称
String apPassword = ""; // AP热点密码

// === WiFi & MQTT ===
const char* ssid = "wifi_ssid";
const char* password = "wifi_password";
char mqtt_server[40] = "0.0.0.0";  //配置为0.0.0.0禁用mqtt
int mqtt_port = 1883;
char mqtt_user[40] = "mqtt_user";
char mqtt_pass[40] = "mqtt_pass";

// === MQTT主题 ===
const char* TOPIC_SET_TEMP = "dryer/control/temp";
const char* TOPIC_SET_TIME = "dryer/control/time";
const char* TOPIC_SET_MODE = "dryer/control/mode";
const char* TOPIC_POWER = "dryer/control/power";
const char* TOPIC_STOP = "dryer/control/stop";
const char* TOPIC_AUTOTUNE = "dryer/control/autotune";
const char* TOPIC_SET_FAN = "dryer/control/fan";
const char* TOPIC_SET_EXHAUST = "dryer/control/exhaust";
const char* TOPIC_SET_EXHAUST_TIME = "dryer/control/exhaust_time";
const char* TOPIC_SET_EXHAUST_MODE = "dryer/control/exhaust_mode";
const char* TOPIC_SET_EXHAUST_FIRST_DELAY = "dryer/control/exhaust_first_delay";
const char* TOPIC_SET_EXHAUST_ON_DURATION = "dryer/control/exhaust_on_duration";
const char* TOPIC_SET_EXHAUST_OFF_DURATION = "dryer/control/exhaust_off_duration";
const char* TOPIC_SET_AUTO_DRY_ENABLE = "dryer/control/auto_dry_enable";
const char* TOPIC_SET_AUTO_DRY_START = "dryer/control/auto_dry_start";
const char* TOPIC_SET_AUTO_DRY_DURATION = "dryer/control/auto_dry_duration";
const char* TOPIC_SET_AUTO_DRY_COOLDOWN = "dryer/control/auto_dry_cooldown";
const char* TOPIC_ALERTS = "dryer/alerts";
const char* TOPIC_STATE_TEMP = "dryer/state/temp";
const char* TOPIC_STATE_TARGET_TEMP = "dryer/state/target_temp";
const char* TOPIC_STATE_CHAMBER_TEMP = "dryer/state/chamber_temp";
const char* TOPIC_STATE_AHT30_TEMP = "dryer/state/aht30_temp";
const char* TOPIC_STATE_AHT30_HUMIDITY = "dryer/state/aht30_humidity";
const char* TOPIC_STATE_TIME = "dryer/state/time";
const char* TOPIC_STATE_REMAINING = "dryer/state/remaining";
const char* TOPIC_STATE_MODE = "dryer/state/mode";
const char* TOPIC_STATE_STATUS = "dryer/state/status";
const char* TOPIC_STATE_PID = "dryer/state/pid";
const char* TOPIC_STATE_IP = "dryer/state/ip";
const char* TOPIC_STATE_FAN = "dryer/state/fan";
const char* TOPIC_STATE_EXHAUST = "dryer/state/exhaust";
const char* TOPIC_STATE_EXHAUST_MODE = "dryer/state/exhaust_mode";
const char* TOPIC_STATE_EXHAUST_TIME = "dryer/state/exhaust_time";
const char* TOPIC_STATE_EXHAUST_DIRECTION = "dryer/state/exhaust_dir";
const char* TOPIC_STATE_EXHAUST_FIRST_DELAY = "dryer/state/exhaust_first_delay";
const char* TOPIC_STATE_EXHAUST_ON_DURATION = "dryer/state/exhaust_on_duration";
const char* TOPIC_STATE_EXHAUST_OFF_DURATION = "dryer/state/exhaust_off_duration";
const char* TOPIC_STATE_AUTO_DRY_ENABLE = "dryer/state/auto_dry_enable";
const char* TOPIC_STATE_AUTO_DRY_START = "dryer/state/auto_dry_start";
const char* TOPIC_STATE_AUTO_DRY_DURATION = "dryer/state/auto_dry_duration";
const char* TOPIC_STATE_AUTO_DRY_COOLDOWN = "dryer/state/auto_dry_cooldown";

// === 预设材料配置 ===
struct MaterialPreset {
  const char* name;
  int temp;
  int time; // 分钟
};
const MaterialPreset PRESETS[] = {
  {"PLA", 55, 360},     //6h
  {"PETG", 65, 480},    //8h
  {"ABS", 80, 480},     //8h
  {"ASA", 80, 480},     //8h
  {"PA", 80, 720}       //12h
};
const int PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

// === 错误处理 ===
String lastErrorMessage = "";
String lastErrorMessage_ch = "";
String lastAutoTuneResult = "";      // "success"/"failed"/empty when no result yet
String lastAutoTuneMessage = "";     // human friendly message for last autotune result
double lastAutoTuneKp = 0;            // last autotune Kp
double lastAutoTuneKi = 0;            // last autotune Ki
double lastAutoTuneKd = 0;            // last autotune Kd
unsigned long errorDisplayTime = 0;
const unsigned long ERROR_DISPLAY_DURATION = 5000;  // 错误显示时间

// === 其他全局状态 ===
enum SystemState { STANDBY, DRYING, PID_AUTOTUNE };
enum AutoTuneState { ATUNE_OFF, ATUNE_START, ATUNE_HEATING, ATUNE_COOLING, ATUNE_DONE };
SystemState currentState = STANDBY;
AutoTuneState autotuneState = ATUNE_OFF;

int fanSpeed = 255; // 风扇速度值 (0-255)

bool mqttEnabled = true;  // 是否启用MQTT连接HA
unsigned long lastWifiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 10000; // 每10秒尝试wifi重连

unsigned long lastNetworkPrint = 0;
unsigned long lastSecondTick = 0; // 每秒计时
const unsigned long COM_PRINT_INTERVAL = 3000; // 串口调试日志打印间隔
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000; // MQTT重连间隔

double targetTemp = 55.0;
int targetTime = 360; // 分钟
int remainingTime = 0; // 秒
int currentPreset = 1; // 0=自定义, 1=PLA...
double currentTemp = 0.0; // 当前温度（主传感器）
double chamberTemp = NAN;  //仓温（默认无效）
double pidOutput = 0.0;
bool chamberSensorConnected = false; // 是否连接仓温传感器
double pidAutotuneTarget = PID_AUTOTUNE_TARGET; // 可配置的PID整定目标温度

double pidInputTemp = 0.0;          // PID使用的滤波后温度
double pidSetpointTemp = 55.0;      // PID内部使用的平滑设定值
unsigned long lastPidUpdate = 0;    // PID上次更新时刻
double pidOutMinBackup = MIN_PWM;   // 进入自动整定前备份的输出下限
double pidOutMaxBackup = 255;       // 进入自动整定前备份的输出上限
bool pidWasAuto = false;            // 进入自动整定前PID模式

sensors_event_t aht_humidity, aht_temp;  // 温湿度计模块变量
float aht30Temperature = 0.0;
float aht30Humidity = 0.0;
bool aht30Connected = false;

// === 湿度自动烘干 ===
bool autoDryEnabled = false;            // 是否根据湿度自动开启/停止烘干
float autoDryStartHumidity = 30.0f;     // 开始烘干湿度阈值(%)
int autoDryRunMinutes = 240;             // 自动烘干运行时长(分钟，>0)
bool autoDryRunning = false;            // 自动烘干是否处于本轮运行中
int autoDryPrevTargetTime = 0;          // 保存手动设定时间，自动烘干结束后恢复
int autoDryCooldownMinutes = 30;        // 用户停止后禁用自动烘干的时长(分钟)
unsigned long autoDryBlockUntil = 0;    // 自动烘干禁止触发的截止时间戳(ms)

String savedScanResult = "";   // 存储预扫描的WiFi-ap模式
bool scanCompleted = false;    // 是否完成预扫描
String staScanResult = "[]";   // 存储STA模式的wifi扫描结果
bool staScanRequested = false; // STA模式扫描请求标志
bool showingAPInfo = false;    // 控制AP模式下显示信息状态
unsigned long lastDisplaySwitch = 0; // 上一次AP模式显示切换时间
const unsigned long SYSTEM_INFO_DURATION = 5000; // 系统信息显示时间
const unsigned long AP_INFO_DURATION = 5000;     // AP信息显示时间

bool updateInProgress = false;  //固件更新状态
size_t updateSize = 0;

unsigned long lastTempRead = 0;
unsigned long lastUpdateTime = 0;
bool fanRunning = false;
bool heaterActive = false;
bool safetyShutdown = false;  // 安全保护状态标志
double safetyResumeTemp = 0.0; // 安全恢复温度

// === 排气阀控制 ===
bool exhaustValveOpen = false;                 // 排气阀当前状态
bool exhaustMotorReversed = false;             // 电机方向是否反转
const unsigned long EXHAUST_CLOSE_EXTRA_MS = 1500; // 关闭额外延时(ms)
unsigned long exhaustActionTime = 12000;        // 排气阀动作时间(ms)
unsigned long exhaustActionStart = 0;          // 排气阀动作开始时间
bool exhaustActionInProgress = false;          // 排气阀动作是否进行中
String exhaustActionType = "";                 // 当前动作类型："open"  "close"
unsigned long dryingStartTime = 0;             // 烘干开始时间
bool exhaustCycleActive = false;               // 排气循环是否激活
unsigned long lastExhaustCycleTime = 0;        // 上次排气循环时间
bool exhaustCycleState = false;                // 排气循环当前状态：true开, false关
bool exhaustAutoMode = true;                   // 排气阀自动控制标志
bool exhaustFirstOpenDone = false;             // 首次开启是否已执行
unsigned long exhaustFirstDelay = 30UL * 60UL * 1000UL; // 首次开启延迟(ms)
unsigned long exhaustOnDuration = 5UL * 60UL * 1000UL; // 开启持续时间(ms)
unsigned long exhaustOffDuration = 25UL * 60UL * 1000UL; // 关闭持续时间(ms)

// 排气阀相关的NVS键名（ESP32 NVS键名上限15字符，缩短以确保写入成功）
const char* PREF_KEY_EXH_ACTION_TIME = "exh_act_ms";    //排气阀动作时间
const char* PREF_KEY_EXH_MOTOR_REV = "exh_dir_rev";     //排气阀电机方向反转
const char* PREF_KEY_EXH_FIRST_DELAY = "exh_first_ms";  //烘干期间首次开启延时
const char* PREF_KEY_EXH_ON_DURATION = "exh_on_ms";     //烘干期间开启持续时间
const char* PREF_KEY_EXH_OFF_DURATION = "exh_off_ms";   //烘干期间关闭持续时间
const char* PREF_KEY_CURRENT_PRESET = "currentPreset";  // 当前预设编号
const char* PREF_KEY_AUTO_DRY_COOLDOWN = "autoDryCoolMin"; // 自动烘干禁用时长(分钟)

// === 温度历史记录 ===
const int HISTORY_SIZE_web = 300; // 5分钟历史数据（每秒1条）
struct TempHistory_web {
  float mainTemp_web;
  float chamberTemp_web;
  unsigned long timestamp_web;
};
TempHistory_web tempHistory_web[HISTORY_SIZE_web];
int historyIndex_web = 0;
unsigned long lastHistoryUpdate_web = 0;

// PID自动整定结构体
struct AutoTuneData {
  bool isPeak;
  float amplitude;
  unsigned long periodStart;
  unsigned long lastPeakTime;
  float lastPeakTemp;
  float lastValleyTemp;
  float amplitudes[PID_AUTOTUNE_MAX_CYCLES * 2];
  unsigned long periods[PID_AUTOTUNE_MAX_CYCLES * 2];
  size_t count;
};
AutoTuneData autotuneData;

// PID自动整定变量
unsigned long autotuneStartTime = 0;
unsigned long autotuneSampleTime = 0;
bool output = HIGH;

// === PID 控制器 ===
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
double originalKp = 0, originalKi = 0, originalKd = 0; // 存储原始PID参数
PID heaterPID(&pidInputTemp, &pidOutput, &pidSetpointTemp, Kp, Ki, Kd, DIRECT);

Preferences preferences;
String lastResetReason = "";

String resetReasonToString(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN: return "未知重启原因 (ESP_RST_UNKNOWN)";
    case ESP_RST_POWERON: return "上电复位 (ESP_RST_POWERON)";
    case ESP_RST_EXT: return "外部复位 (ESP_RST_EXT)";
    case ESP_RST_SW: return "软件重启 (ESP_RST_SW)";
    case ESP_RST_PANIC: return "Panic 异常 (ESP_RST_PANIC)";
    case ESP_RST_INT_WDT: return "内部看门狗 (ESP_RST_INT_WDT)";
    case ESP_RST_TASK_WDT: return "任务看门狗 (ESP_RST_TASK_WDT)";
    case ESP_RST_WDT: return "其他看门狗 (ESP_RST_WDT)";
    case ESP_RST_DEEPSLEEP: return "从深度睡眠唤醒 (ESP_RST_DEEPSLEEP)";
    case ESP_RST_BROWNOUT: return "欠压复位 (ESP_RST_BROWNOUT)";
    case ESP_RST_SDIO: return "SDIO 复位 (ESP_RST_SDIO)";
    default: return "其他/未识别重启原因";
  }
}

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 函数声明
void setupPins();
void setupDisplay();
void setupAht();
void setupWiFi();
void setupMQTT();
bool reconnectWiFi();
void reconnectMQTT();
void readTemperature();
void updateDisplay();
void startDrying();
void stopDrying(bool userStop = false);
void updateSystemState();
void controlFan();
void handleMQTT(char* topic, byte* payload, unsigned int length);
void publishState();
void safetyCheck();
void startAutoTune();
void handleAutoTune();
void finishAutoTune(bool success);
float filteredTemperature(float newTemp);
bool checkStability();
void displayAutotuneResult(double Kp, double Ki, double Kd);
void updateAutoTuneDisplay();
void drawGraph();
void resetTempHistory();
void setErrorMessage(String message, String ch_message);
void clearErrorMessage();
void drawErrorMessage();
void startAPMode();
void handleRoot();
void handleSaveConfig();
void displayAPInfo();
void handleScanRequest();
String scanWiFiNetworks();
void handleSaveMQTTConfig();
void handleMQTTConfigPage();
void handleRootPage();
void handleFactoryReset();
void handleReboot();
void handleControl();
void handleApiStatus();
void handleClearAlert();
String formatTime(int totalSeconds);
void handleWifiConfigPage();
void handleScanWifi();
void handleSaveSTAConfig();
void handleTriggerScan();
void staScanNetworks();
void handleFirmwareUpdatePage();
void handleFirmwareUpdate();
void handleTempUpdate();
void handleButtons();
void controlExhaustValve(bool open);
void stopExhaustMotor();
void updateExhaustValve();
void handleExhaustCycle();
void setExhaustActionTime(unsigned long timeMs);
bool setExhaustFirstDelayMinutes(unsigned long minutes);
bool setExhaustOnDurationMinutes(unsigned long minutes);
bool setExhaustOffDurationMinutes(unsigned long minutes);
void toggleExhaustMotorDirection();
void openExhaustValve();
void closeExhaustValve();
String getExhaustStateCode();
String getExhaustStateLabel();
void recordAutoTuneCycle(float amplitude, unsigned long periodMs);
void handleAutoDry();
bool setAutoDryCooldownMinutes(int minutes);
bool setAutoDryEnabled(bool enabled);
bool setAutoDryStart(float humidity);
bool setAutoDryRunMinutes(int minutes);
bool setPidAutotuneTarget(float target);
void handleApiPerformance();
float getCpuUsage();
void publishPerformanceData(float cpuUsage, float ramUsage, float ramUsed, float ramTotal, float psramUsage, float psramUsed, float psramTotal, float flashUsage, float flashUsed, float flashTotal);
void publishCurrentPerformanceData();

void setup() {
  Serial.begin(115200);

  setupPins();
  setupDisplay();
  setupAht();
  sensors.begin();
  sensors.setWaitForConversion(false);
  chamberSensor.begin();
  chamberSensor.setWaitForConversion(false);
  pixels.begin();
  pixels.clear();

  // 加载配置
  preferences.begin("dryer", false);

  // 记录本次启动的重启原因并保存到Preferences，供网页展示
  esp_reset_reason_t reason = esp_reset_reason();
  lastResetReason = resetReasonToString(reason);
  preferences.putString("last_reset_reason", lastResetReason);

  // 保存版本号
  if (!preferences.isKey("fw_version")) {
    #ifdef FIRMWARE_VERSION
      preferences.putString("fw_version", FIRMWARE_VERSION);
    #else
      preferences.putString("fw_version", "0.0.0");
    #endif
  }
  String currentVersion = preferences.getString("fw_version", "未知");
  Serial.print("Firmware Version: ");
  Serial.println(currentVersion);
  if(currentVersion != FIRMWARE_VERSION){
    preferences.putString("fw_version", FIRMWARE_VERSION);
  }

  // 排气阀配置
  if (preferences.isKey(PREF_KEY_EXH_ACTION_TIME)) {
    exhaustActionTime = preferences.getULong(PREF_KEY_EXH_ACTION_TIME, exhaustActionTime);
  }
  if (preferences.isKey(PREF_KEY_EXH_MOTOR_REV)) {
    exhaustMotorReversed = preferences.getBool(PREF_KEY_EXH_MOTOR_REV, false);
  }
  if (preferences.isKey("exhaustAutoMode")) {
    exhaustAutoMode = preferences.getBool("exhaustAutoMode", true);
  }
  if (preferences.isKey(PREF_KEY_EXH_FIRST_DELAY)) {
    exhaustFirstDelay = preferences.getULong(PREF_KEY_EXH_FIRST_DELAY, exhaustFirstDelay);
  }
  if (preferences.isKey(PREF_KEY_EXH_ON_DURATION)) {
    exhaustOnDuration = preferences.getULong(PREF_KEY_EXH_ON_DURATION, exhaustOnDuration);
  }
  if (preferences.isKey(PREF_KEY_EXH_OFF_DURATION)) {
    exhaustOffDuration = preferences.getULong(PREF_KEY_EXH_OFF_DURATION, exhaustOffDuration);
  }

  // 自动烘干配置
  if (preferences.isKey("autoDryEnabled")) {
    autoDryEnabled = preferences.getBool("autoDryEnabled", false);
  }
  if (preferences.isKey("autoDryStartH")) {
    autoDryStartHumidity = preferences.getFloat("autoDryStartH", autoDryStartHumidity);
  }
  if (preferences.isKey("autoDryRunMin")) {
    autoDryRunMinutes = preferences.getInt("autoDryRunMin", autoDryRunMinutes);
    if (autoDryRunMinutes <= 0) autoDryRunMinutes = 1;
  }
  if (preferences.isKey(PREF_KEY_AUTO_DRY_COOLDOWN)) {
    autoDryCooldownMinutes = preferences.getInt(PREF_KEY_AUTO_DRY_COOLDOWN, autoDryCooldownMinutes);
    if (autoDryCooldownMinutes < 0) autoDryCooldownMinutes = 0;
    if (autoDryCooldownMinutes > 1440) autoDryCooldownMinutes = 1440;
  }
  // 重新校验范围
  setAutoDryStart(autoDryStartHumidity);
  setAutoDryCooldownMinutes(autoDryCooldownMinutes);

  // PID自动整定目标温度
  if (preferences.isKey("pidAutoTarget")) {
    pidAutotuneTarget = preferences.getDouble("pidAutoTarget", pidAutotuneTarget);
  }
  setPidAutotuneTarget(pidAutotuneTarget); // 归一化范围并写回有效值

  // 读取上次保存的温度/时间设定
  if (preferences.isKey("targetTemp")) {
    double savedTemp = preferences.getDouble("targetTemp", targetTemp);
    if (savedTemp < MIN_TEMP) savedTemp = MIN_TEMP;
    if (savedTemp > MAX_TEMP) savedTemp = MAX_TEMP;
    targetTemp = savedTemp;
  }
  if (preferences.isKey("targetTime")) {
    int savedTime = preferences.getInt("targetTime", targetTime);
    if (savedTime < 0) savedTime = 0;
    if (savedTime > 1440) savedTime = 1440;
    targetTime = savedTime;
  }

  // 读取当前预设编号
  if (preferences.isKey(PREF_KEY_CURRENT_PRESET)) {
    int savedPreset = preferences.getInt(PREF_KEY_CURRENT_PRESET, currentPreset);
    if (savedPreset < 0) savedPreset = 0;
    if (savedPreset > PRESET_COUNT) savedPreset = PRESET_COUNT;
    currentPreset = savedPreset;
  }

  // 尝试读取风速参数
  if (preferences.isKey("fanSpeed")) {
    fanSpeed = preferences.getInt("fanSpeed");
  }
  // 尝试读取pid参数
  if (preferences.isKey("Kp")) {
    Kp = preferences.getDouble("Kp");
  } else {
    Kp = 2.0;  // 默认值
  }
  if (preferences.isKey("Ki")) {
    Ki = preferences.getDouble("Ki");
  } else {
    Ki = 0.5;
  }
  if (preferences.isKey("Kd")) {
    Kd = preferences.getDouble("Kd");
  } else {
    Kd = 1.0;
  }

  // 初始化PID输入/设定值
  pidSetpointTemp = targetTemp;
  pidInputTemp = targetTemp;
  lastPidUpdate = millis();

  // PID设置
  heaterPID.SetMode(AUTOMATIC);
  heaterPID.SetOutputLimits(MIN_PWM, 255);  // 设置最小PWM值
  heaterPID.SetSampleTime(1000);    // 1秒采样
  heaterPID.SetTunings(Kp, Ki, Kd);

  // 加热器PWM设置
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PWM_PIN, 0);

  setupWiFi();
  setupMQTT();

  // 初始状态
  ledcWrite(HEATER_PWM_CHANNEL, 0); // 关闭加热器
  ledcWrite(FAN_PWM_CHANNEL, 0);    // 关风扇
  //ledcWrite(FAN_PWM_CHANNEL, fanSpeed);// 开风扇

  // 初始化温度历史-PID用
  resetTempHistory();
  // 初始化温度历史数据-WEB用
  for (int i = 0; i < HISTORY_SIZE_web; i++) {
    tempHistory_web[i].mainTemp_web = 0;
    tempHistory_web[i].chamberTemp_web = 0;
    tempHistory_web[i].timestamp_web = 0;
  }
}

void loop() {
  // 处理Web请求
  server.handleClient();
  handleButtons();

  unsigned long currentMillis = millis();
  if (millis() - lastSecondTick >= 1000) {
      lastSecondTick = millis();
      // 只有在烘干状态下才减少时间
      if (currentState == DRYING) {
        if (remainingTime > 0) {
            remainingTime--;
        } else if(remainingTime == 0){
            stopDrying();
        }
      }
      if(apMode){
        if (autotuneState == ATUNE_OFF){
          if ((showingAPInfo && currentMillis - lastDisplaySwitch >= AP_INFO_DURATION) ||
              (!showingAPInfo && currentMillis - lastDisplaySwitch >= SYSTEM_INFO_DURATION)) {
            showingAPInfo = !showingAPInfo;
            lastDisplaySwitch = currentMillis;
            if (showingAPInfo) {
                displayAPInfo();
            } else {
                updateDisplay();
            }
          }
        }
      }else{
        if (autotuneState == ATUNE_OFF) {
          updateDisplay();
        }
      }
  }
  
  // 定期发布性能数据到HA
  static unsigned long lastPerformancePublish = 0;
  if (currentMillis - lastPerformancePublish >= 1000) {
    lastPerformancePublish = currentMillis;
    publishCurrentPerformanceData();
  }

  // 更新排气阀状态处理排气循环
  updateExhaustValve();
  if (currentState == DRYING && exhaustAutoMode) {
    // 首次开启只执行一次，之后进入常规循环
    if (!exhaustFirstOpenDone && currentMillis - dryingStartTime >= exhaustFirstDelay && !exhaustActionInProgress) {
      openExhaustValve();
      exhaustCycleState = true;
      exhaustFirstOpenDone = true;
      lastExhaustCycleTime = currentMillis;
    } else if (exhaustCycleActive && exhaustFirstOpenDone) {
      handleExhaustCycle();
    }
  }

  // 读取温度
  if (currentMillis - lastTempRead > 300) {
    readTemperature();
    lastTempRead = currentMillis;
    handleAutoDry();
    // 记录温度历史
    if (autotuneState != ATUNE_OFF) {
      tempHistory[historyIndex] = currentTemp;
      historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    }
  }

  // 间隔时间更新系统状态
  if (currentMillis - lastUpdateTime > 200) {
    publishState();
    // 处理PID自动整定
    if (autotuneState != ATUNE_OFF) {
      handleAutoTune();
    } else {
      updateSystemState();
    }
    lastUpdateTime = currentMillis;
  }

  // MQTT处理
  if (mqttEnabled) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    } else {
      mqttClient.loop();
      if(lastErrorMessage == "MQTT connect failed"){
        clearErrorMessage();
      }
    }
  }else{
    if(lastErrorMessage == "MQTT connect failed"){
      clearErrorMessage();
    }
  }

  // WIFI处理
  if (!apMode && WiFi.status() != WL_CONNECTED) {
    if (currentMillis - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
      lastWifiReconnectAttempt = currentMillis;
      reconnectWiFi();
    }
  }

  // STA模式WiFi扫描请求
  if (staScanRequested) {
    staScanNetworks();  // 执行扫描
    staScanRequested = false;
  }

  // 安全监测
  if (autotuneState == ATUNE_OFF) {
    safetyCheck();
  }
}

// 绘制温度曲线图
void drawGraph() {
  // 坐标系参数
  const int graphX = 0;
  const int graphY = 28;
  const int graphWidth = 128;
  const int graphHeight = 36;
  
  // 动态计算温度范围，确保包含历史/当前值，避免曲线掉出边界
  float minDisplayTemp = pidAutotuneTarget - 10.0;
  float maxDisplayTemp = pidAutotuneTarget + 10.0;
  float observedMin = NAN;
  float observedMax = NAN;

  // 扫描历史有效点，记录最小/最大
  for (int i = 0; i < HISTORY_SIZE; i++) {
    float t = tempHistory[i];
    if (isnan(t)) continue;
    if (isnan(observedMin) || t < observedMin) observedMin = t;
    if (isnan(observedMax) || t > observedMax) observedMax = t;
  }

  // 将当前温度纳入范围
  if (!isnan(currentTemp)) {
    if (isnan(observedMin) || currentTemp < observedMin) observedMin = currentTemp;
    if (isnan(observedMax) || currentTemp > observedMax) observedMax = currentTemp;
  }

  // 扩展显示范围，给上下各留1度余量
  if (!isnan(observedMin)) {
    minDisplayTemp = min(minDisplayTemp, observedMin - 1.0f);
    maxDisplayTemp = max(maxDisplayTemp, observedMax + 1.0f);
  }

  // 确保最小跨度，避免除0
  if (maxDisplayTemp - minDisplayTemp < 5.0f) {
    float center = (maxDisplayTemp + minDisplayTemp) * 0.5f;
    minDisplayTemp = center - 2.5f;
    maxDisplayTemp = center + 2.5f;
  }
  
  // 绘制坐标轴
  display.drawRect(graphX, graphY, graphWidth, graphHeight, SSD1306_WHITE);
  
  // 绘制设定温度线
  int targetY = graphY + graphHeight - (int)((pidAutotuneTarget - minDisplayTemp) * graphHeight / (maxDisplayTemp - minDisplayTemp));
  if (targetY < graphY) targetY = graphY;
  if (targetY > graphY + graphHeight) targetY = graphY + graphHeight;
  
  // 虚线样式
  for (int x = graphX; x < graphX + graphWidth; x += 4) {
    display.drawPixel(x, targetY, SSD1306_WHITE);
  }
  
  // 标记目标温度（向左适当偏移以避免越界换行）
  display.setCursor(graphX + graphWidth - 40, targetY - 8);
  display.print(pidAutotuneTarget, 1);
  display.print("C");
 
  // 绘制温度曲线
  int lastX = graphX;
  int lastY = graphY + graphHeight; // 初始位置在底部
  
  for (int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex + i) % HISTORY_SIZE;
    float temp = tempHistory[idx];
    
    // 跳过无效点
    if (isnan(temp)) continue;
    
    int x = graphX + (i * graphWidth / HISTORY_SIZE);
    int y = graphY + graphHeight - (int)((temp - minDisplayTemp) * graphHeight / (maxDisplayTemp - minDisplayTemp));
    
    // 确保点在图形范围内
    if (y < graphY) y = graphY;
    if (y > graphY + graphHeight) y = graphY + graphHeight;
    
    // 绘制点
    display.drawPixel(x, y, SSD1306_WHITE);
    
    // 绘制线（从上一个有效点）
    if (i > 0 && !isnan(tempHistory[(idx-1+HISTORY_SIZE) % HISTORY_SIZE])) {
      display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
    }
    
    lastX = x;
    lastY = y;
  }
  
  // 标记当前温度点（仅绘制标记，不再在图中重复显示数值）
  if (!isnan(currentTemp)) {
    int x = graphX + graphWidth - 5; // 最右边
    int y = graphY + graphHeight - (int)((currentTemp - minDisplayTemp) * graphHeight / (maxDisplayTemp - minDisplayTemp));
    if (y < graphY) y = graphY;
    if (y > graphY + graphHeight) y = graphY + graphHeight;
    
    // 绘制圆形标记
    display.fillCircle(x, y, 2, SSD1306_WHITE);
  }
}

// 重置温度历史
void resetTempHistory() {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    tempHistory[i] = NAN; // 无效值
  }
  historyIndex = 0;
}

// 记录一次振幅/周期
void recordAutoTuneCycle(float amplitude, unsigned long periodMs) {
  if (autotuneData.count >= PID_AUTOTUNE_MAX_CYCLES * 2) return;
  autotuneData.amplitudes[autotuneData.count] = amplitude;
  autotuneData.periods[autotuneData.count] = periodMs;
  autotuneData.count++;
}

// PID自动整定功能
void startAutoTune() {
  if (currentState != STANDBY) return;
  // 重置前一次整定结果，准备新一轮
  lastAutoTuneResult = "";
  lastAutoTuneMessage = "";
  lastAutoTuneKp = 0;
  lastAutoTuneKi = 0;
  lastAutoTuneKd = 0;
  // 初始化自动整定状态
  originalKp = Kp;
  originalKi = Ki;
  originalKd = Kd;
  autotuneState = ATUNE_START;
  currentState = PID_AUTOTUNE;
  autotuneStartTime = millis();
  autotuneSampleTime = autotuneStartTime + PID_AUTOTUNE_SAMPLE_TIME;
  // 初始化自动整定数据结构
  autotuneData = AutoTuneData();
  autotuneData.isPeak = false;
  autotuneData.lastPeakTemp = pidAutotuneTarget - 10.0;
  autotuneData.lastValleyTemp = pidAutotuneTarget - 10.0;
  autotuneData.count = 0;

  // 强制关闭常规PID影响，整定阶段仅使用0/255开关控制
  pidWasAuto = heaterPID.GetMode() == AUTOMATIC;
  pidOutMinBackup = MIN_PWM;
  pidOutMaxBackup = 255;
  heaterPID.SetMode(MANUAL);
  heaterPID.SetOutputLimits(0, 255);

  // 设置目标温度
  targetTemp = pidAutotuneTarget;
  heaterActive = true;
  fanRunning = true;
  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);

  // 重置温度历史
  resetTempHistory();

  // 发布状态
  if (mqttEnabled && mqttClient.connected()) {
    mqttClient.publish(TOPIC_STATE_STATUS, "autotune");
  }
  publishState();
}

void handleAutoTune() {
  unsigned long now = millis();
  float filteredTemp = filteredTemperature(currentTemp);
  
  // 检查传感器是否有效
  if (isnan(currentTemp) || currentTemp == DEVICE_DISCONNECTED_C) {
    String errorMsg = "Autotune: Sensor Fail";
    String ch_msg = "PID自动整定失败：出风口温度传感器异常";
    setErrorMessage(errorMsg,ch_msg);
    mqttClient.publish(TOPIC_ALERTS, ch_msg.c_str());
    publishState();
    finishAutoTune(false);
    return;
  }else{
    if(lastErrorMessage == "Autotune: Sensor Fail"){
      clearErrorMessage();
    }
  }
  
  if (mqttEnabled && mqttClient.connected()) {
    // 每次处理循环都发布温度
    char tempStr[10];
    dtostrf(currentTemp, 4, 1, tempStr);
    mqttClient.publish(TOPIC_STATE_TEMP, tempStr);
  }

  // 更新温度历史
  tempHistory[historyIndex] = filteredTemp;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  switch (autotuneState) {
    case ATUNE_START:
      // 加热到目标温度低5℃
      if (currentTemp > pidAutotuneTarget - 5.0) {
        autotuneState = ATUNE_HEATING;
        output = HIGH;
        ledcWrite(HEATER_PWM_CHANNEL, (int)PID_AUTOTUNE_OUTPUT_STEP);
        autotuneData.periodStart = now;
      } else {
        ledcWrite(HEATER_PWM_CHANNEL, 255); // 全功率加热
      }
      break;
      
    case ATUNE_HEATING:
      // 检测到新的峰值
      if (filteredTemp > autotuneData.lastPeakTemp) {
        autotuneData.lastPeakTemp = filteredTemp;
      }
      
      // 超过目标温度+噪声带宽
      if (filteredTemp > pidAutotuneTarget + PID_AUTOTUNE_NOISE_BAND) {
        // 计算本次振幅
        float amplitude = autotuneData.lastPeakTemp - autotuneData.lastValleyTemp;
        
        // 记录峰值数据
        recordAutoTuneCycle(amplitude, now - autotuneData.periodStart);
        autotuneData.lastPeakTime = now;
        autotuneData.isPeak = true;
        
        // 关闭加热器
        output = LOW;
        ledcWrite(HEATER_PWM_CHANNEL, 0);
        autotuneState = ATUNE_COOLING;
        
        // 更新周期开始时间
        autotuneData.periodStart = now;
        autotuneData.lastValleyTemp = filteredTemp; // 重置谷值
      }
      break;
      
    case ATUNE_COOLING:
      // 检测到新的谷值
      if (filteredTemp < autotuneData.lastValleyTemp) {
        autotuneData.lastValleyTemp = filteredTemp;
      }
      
      // 低于目标温度-噪声带宽
      if (filteredTemp < pidAutotuneTarget - PID_AUTOTUNE_NOISE_BAND) {
        // 计算本次振幅
        float amplitude = autotuneData.lastPeakTemp - filteredTemp;
        
        // 记录谷值数据
        recordAutoTuneCycle(amplitude, now - autotuneData.periodStart);
        autotuneData.isPeak = false;
        
        // 重新加热
        output = HIGH;
        ledcWrite(HEATER_PWM_CHANNEL, (int)PID_AUTOTUNE_OUTPUT_STEP);
        autotuneState = ATUNE_HEATING;
        
        // 更新周期开始时间
        autotuneData.periodStart = now;
        autotuneData.lastPeakTemp = filteredTemp; // 重置峰值
        
        // 检查是否完成足够周期
        if (autotuneData.count >= PID_AUTOTUNE_MIN_CYCLES * 2) {
          // 检查稳定性
          if (checkStability()) {
            finishAutoTune(true);
            return;
          }
        }
        
        // 检查最大周期数
        if (autotuneData.count >= PID_AUTOTUNE_MAX_CYCLES * 2) {
          finishAutoTune(true);
          return;
        }
      }
      break;
  }
  
  // 超时判断
  if (now - autotuneStartTime > PID_AUTO_TIME_OUT) {
    String errorMsg = "Autotune Timeout";
    String ch_msg = "PID自动整定超时";
    setErrorMessage(errorMsg,ch_msg);
    mqttClient.publish(TOPIC_ALERTS, ch_msg.c_str());
    publishState();
    finishAutoTune(false);
  }else{
    if(lastErrorMessage == "Autotune Timeout"){
      clearErrorMessage();
    }
  }
  
  // 更新显示
  updateAutoTuneDisplay();
}
 
bool checkStability() {
  // 需要至少3个完整周期（6个半周期点）才能检查稳定性
  if (autotuneData.count < 6) return false;
  
  // 计算最后3个周期的平均振幅和周期
  float sumAmplitude = 0.0;
  unsigned long sumPeriod = 0;
  int count = 0;
  
  // 从后往前取3个完整周期（6个点）
  for (int i = (int)autotuneData.count - 1; i >= (int)autotuneData.count - 6; i--) {
    sumAmplitude += autotuneData.amplitudes[i];
    sumPeriod += autotuneData.periods[i];
    count++;
  }
  
  float avgAmplitude = sumAmplitude / count;
  unsigned long avgPeriod = sumPeriod / count;
  
  // 检查振幅稳定性
  for (int i = (int)autotuneData.count - 1; i >= (int)autotuneData.count - 6; i--) {
    float deviation = fabs(autotuneData.amplitudes[i] - avgAmplitude) / avgAmplitude;
    if (deviation > STABILITY_THRESHOLD) {
      return false;
    }
  }
  
  // 检查周期稳定性
  for (int i = (int)autotuneData.count - 1; i >= (int)autotuneData.count - 6; i--) {
    float deviation = fabs((float)(autotuneData.periods[i] - avgPeriod)) / avgPeriod;
    if (deviation > STABILITY_THRESHOLD) {
      return false;
    }
  }
  
  // 振幅在合理范围内
  if (avgAmplitude < MIN_AMPLITUDE || avgAmplitude > MAX_AMPLITUDE) {
    return false;
  }
  
  return true;
}

void finishAutoTune(bool success) {
  // 关闭加热器和风扇
  heaterActive = false;
  ledcWrite(HEATER_PWM_CHANNEL, 0);
  
  if (success) {
    // 计算平均振幅和周期
    float sumAmplitude = 0.0;
    unsigned long sumPeriod = 0;
    // 跳过前两个不稳定周期
    int totalEntries = (int)autotuneData.count;
    int startIndex = min(4, totalEntries);
    int count = 0;

    for (int i = startIndex; i < totalEntries; i++) {
      sumAmplitude += autotuneData.amplitudes[i];
      sumPeriod += autotuneData.periods[i];
      count++;
    }

    if (count < 3) {
      success = false;
    } else {
      float avgAmplitude = sumAmplitude / count;
      unsigned long avgPeriod = sumPeriod / count;

      if (avgAmplitude <= 0.0 || avgPeriod == 0 || !isfinite(avgAmplitude)) {
        success = false;
      } else {
        // 计算临界增益(Ku)和临界周期(Pu)
        float Ku = (4.0f * PID_AUTOTUNE_OUTPUT_STEP) / (PI * avgAmplitude);
        float Pu = avgPeriod / 1000.0f; // 转换为秒

        Kp = 0.45f * Ku;
        Ki = 0.54f * Ku / Pu;
        Kd = 0.06f * Ku * Pu;

        if (!isfinite(Kp) || !isfinite(Ki) || !isfinite(Kd)) {
          success = false;
        } else {
          // 设置新参数
          heaterPID.SetTunings(Kp, Ki, Kd);

          // 保存参数
          preferences.putDouble("Kp", Kp);
          preferences.putDouble("Ki", Ki);
          preferences.putDouble("Kd", Kd);

          // 显示结果
          displayAutotuneResult(Kp, Ki, Kd);

          // 发布PID参数
          char pidStr[30];
          snprintf(pidStr, sizeof(pidStr), "%.2f-%.4f-%.2f", Kp, Ki, Kd);
          if (mqttEnabled && mqttClient.connected()) {
            mqttClient.publish(TOPIC_STATE_PID, pidStr);
          }

          // 记录整定成功结果，便于前端提示
          lastAutoTuneResult = "success";
          lastAutoTuneMessage = "PID自动整定成功，参数已自动保存";
          lastAutoTuneKp = Kp;
          lastAutoTuneKi = Ki;
          lastAutoTuneKd = Kd;
        }
      }
    }
  }

  if (!success) {
    // 若没有明确错误信息，则给出默认失败原因
    String reason = lastErrorMessage_ch.length() ? lastErrorMessage_ch : String("手动停止PID整定或未知错误");
    if (lastErrorMessage == "") {
      setErrorMessage("Autotune: Not enough cycles", reason);
    }

    lastAutoTuneResult = "failed";
    lastAutoTuneMessage = reason;
    lastAutoTuneKp = 0;
    lastAutoTuneKi = 0;
    lastAutoTuneKd = 0;

    // 恢复原始PID参数
    heaterPID.SetTunings(originalKp, originalKi, originalKd);
  }

  // 风扇设置
  fanRunning = false;
  ledcWrite(FAN_PWM_CHANNEL, 255);
  
  // 结束状态
  autotuneState = ATUNE_OFF;
  currentState = STANDBY;

  // 恢复进入自动整定前的PID模式与输出限制
  heaterPID.SetOutputLimits(pidOutMinBackup, pidOutMaxBackup);
  heaterPID.SetMode(pidWasAuto ? AUTOMATIC : MANUAL);

  // 将最新结果推送给前端
  publishState();
  
  // 添加短暂延迟让用户看到结果
  delay(1500);
}

void displayAutotuneResult(double Kp, double Ki, double Kd) {
  display.clearDisplay();
  drawZh(9,0,PID_auto_done_110x40,display);
  display.setCursor(30, 39);
  display.print("Kp: "); display.print(Kp, 2);
  display.setCursor(30, 48);
  display.print("Ki: "); display.print(Ki, 4);
  display.setCursor(30, 56);
  display.print("Kd: "); display.print(Kd, 2);
  display.display();
}

void updateAutoTuneDisplay() {
  display.clearDisplay();
  
  // 显示自动整定状态
  drawZh(89, 0, PID_auto_39x26, display);
  
  // 显示当前温度
  drawZh(0, 0, temp_30x13, display);
  display.setCursor(33, 3);
  display.print(currentTemp, 1);
  display.print("C");
  
  // 显示运行时间
  drawZh(0, 14, time_30x13, display);
  display.setCursor(33, 17);
  unsigned long elapsed = (millis() - autotuneStartTime) / 1000;
  display.print(elapsed);
  display.print("s");
  
  // 显示周期计数
  display.setCursor(80, 17);
  display.print("Cyc:");
  display.print(autotuneData.count / 2);
  
  // 绘制温度曲线
  drawGraph();
  
  display.display();
}

// 温度滤波函数
float filteredTemperature(float newTemp) {
  static float filtered = NAN;
  
  if (isnan(filtered)) {
    filtered = newTemp;
  } else {
    filtered = (1.0 - TEMP_FILTER_WEIGHT) * filtered + TEMP_FILTER_WEIGHT * newTemp;
  }
  
  return filtered;
}

void setupPins() {
  // 风扇PWM
  ledcSetup(FAN_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0); // 初始关风扇

  // 按键
  pinMode(TEMP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIME_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FACTORY_BUTTON_PIN, INPUT_PULLUP);

  // 排气阀控制
  pinMode(EXHAUST_INA_PIN, OUTPUT);
  pinMode(EXHAUST_INB_PIN, OUTPUT);
  digitalWrite(EXHAUST_INA_PIN, LOW);
  digitalWrite(EXHAUST_INB_PIN, LOW);
}

void setupDisplay() {
  int16_t x1, y1;
  uint16_t w, h;
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  drawZh(1,2,device_name_125x25,display);
  #ifdef FIRMWARE_VERSION
    String versionText = "V:" + String(FIRMWARE_VERSION);
  #else
    String versionText = "V: Unknown Version";
  #endif
  display.getTextBounds(versionText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 28);
  display.print(versionText);
  drawZh(31,47,starting_65x16,display);
  display.setTextSize(1);
  display.display();
  display.clearDisplay();
}

void setupAht(){
  Wire1.begin(AHT30_SDA_PIN, AHT30_SCL_PIN);
  if (aht.begin(&Wire1)) {
    Serial.println("AHT30 successfully");
    aht30Connected = true;
  } else {
    Serial.println("Failed to AHT30");
    aht30Connected = false;
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  //先扫描一次wifi
  staScanNetworks();
  // 尝试读取存储的WiFi配置
  String savedSSID = "";
  String savedPass = "";
  if (preferences.isKey("wifi_ssid")) {
    savedSSID = preferences.getString("wifi_ssid", "");
  }
  if (preferences.isKey("wifi_password")) {
    savedPass = preferences.getString("wifi_password", "");
  }
  if (savedSSID != "") {
    ssid = savedSSID.c_str();
    if (savedPass == "") {
      password = NULL;
    } else {
      password = savedPass.c_str();
    }
  } 

  if (password == NULL) {
    WiFi.begin(ssid); // 开放式网络
  } else {
    WiFi.begin(ssid, password); // 加密网络
  }
  unsigned long startTime = millis();
  display.clearDisplay();
  drawZh(4,12,connecting_wifi_120x20,display);
  display.setCursor(0, 42);
  display.print("SSID: ");
  display.print(ssid);
  display.display();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    display.print(".");
    display.display();
    
    // 超时处理
    if (millis() - startTime > WIFI_TIMEOUT) {
      display.clearDisplay();
      drawZh(4,12,connect_wifi_timeout_120x20,display);
      drawZh(4,32,will_start_ap_120x20,display);
      display.display();
      delay(2000);
      startAPMode(); // AP模式
      return;
    }
  }

  // WiFi连接成功后启动Web服务器
  server.on("/", handleRootPage);          // 设备状态页
  server.on("/mqtt-config", handleMQTTConfigPage); // MQTT配置页
  server.on("/save-mqtt", HTTP_POST, handleSaveMQTTConfig); //MQTT保存
  server.on("/factory-reset", HTTP_POST, handleFactoryReset); //恢复出厂设置
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/wifi-config", HTTP_GET, handleWifiConfigPage);
  server.on("/scan-wifi", HTTP_GET, handleScanWifi);
  server.on("/trigger-scan", HTTP_GET, handleTriggerScan);
  server.on("/save-sta-config", HTTP_POST, handleSaveSTAConfig);
  server.on("/firmware-update", HTTP_GET, handleFirmwareUpdatePage);
  server.on("/update-firmware", HTTP_POST, []() { server.send(200, "text/plain", "上传完成"); },handleFirmwareUpdate);
  server.on("/api/temp", HTTP_GET, handleTempUpdate);
  // 提供清除重启原因的接口（前端点击关闭时调用）
  server.on("/clear-reset", HTTP_POST, [](){
    preferences.remove("last_reset_reason");
    lastResetReason = "";
    server.send(200, "text/plain", "ok");
  });
  server.on("/clear-alert", HTTP_POST, handleClearAlert);  // 清除告警信息
  server.on("/api/performance", HTTP_GET, handleApiPerformance);  // 性能数据
  server.begin();

  // 清除任何先前的错误消息
  lastErrorMessage = "";
}

// === 连接wifi成功后本地页面 ===
void handleRootPage() {
  String html = R"=====(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, viewport-fit=cover">
  <title>AMS-Lite烘干器控制面板</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }

    html {
      -webkit-text-size-adjust: 100%; /* 防止移动端自动放大字体导致布局挤压 */
    }
    
    body {
      background: linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%);
      min-height: 100vh;
      padding: 10px;
      color: #2c3e50;
    }
    
    .toast {
      position: fixed;
      top: 20px;
      left: 50%;
      transform: translateX(-50%);
      background-color: #e74c3c;
      color: white;
      padding: 15px 30px;
      border-radius: 5px;
      box-shadow: 0 4px 10px rgba(0,0,0,0.2);
      z-index: 2000;
      opacity: 0;
      transition: opacity 0.3s;
    }
    
    .toast.show {
      opacity: 1;
    }

    .btn-disabled {
      opacity: 0.5;
      cursor: not-allowed !important;
    }
    
    .preset-btn.disabled {
      opacity: 0.5;
      cursor: not-allowed !important;
    }

    .tooltip {
      position: relative;
      display: inline-block;
    }
    
    .tooltip .tooltiptext {
      visibility: hidden;
      width: 180px;
      background-color: #555;
      color: #fff;
      text-align: center;
      border-radius: 6px;
      padding: 8px;
      position: absolute;
      z-index: 1;
      bottom: 125%;
      left: 50%;
      transform: translateX(-50%);
      opacity: 0;
      transition: opacity 0.3s;
      font-size: 0.9rem;
    }
    
    .tooltip .tooltiptext::after {
      content: "";
      position: absolute;
      top: 100%;
      left: 50%;
      margin-left: -5px;
      border-width: 5px;
      border-style: solid;
      border-color: #555 transparent transparent transparent;
    }
    
    .tooltip:hover .tooltiptext {
      visibility: visible;
      opacity: 1;
    }

    .temp-control {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 15px;
    }
    
    .temp-btn {
      width: 50px;
      height: 50px;
      border-radius: 50%;
      background: #3498db;
      color: white;
      font-size: 1.1rem;
      font-weight: bold;
      border: none;
      cursor: pointer;
      transition: all 0.3s;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 0;
      line-height: 1;
      text-align: center;
    }
    
    .temp-btn:hover {
      background: #2980b9;
      transform: scale(1.1);
    }
    
    .temp-display {
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .fan-display {
      font-size: 1.8rem;
      font-weight: 600;
      min-width: 120px;
      text-align: center;
      color: #3498db;
    }

    .container {
      width: 98%;
      max-width: 1800px;
      margin: 0 auto;
      background: white;
      border-radius: 15px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.1);
      overflow: hidden;
      display: flex;
      flex-direction: column;
      min-height: calc(100vh - 20px);
    }
    
    header {
      background: #3498db;
      color: white;
      padding: 18px 22px;
    }

    .header-inner {
      display: grid;
      grid-template-columns: auto 1fr auto;
      align-items: center;
      gap: 16px;
      width: 100%;
    }

    .header-title {
      display: flex;
      justify-content: center;
      align-items: baseline;
      gap: 12px;
      flex-wrap: nowrap;
      text-align: center;
    }
    
    h1 {
      font-size: 2.2rem;
      margin-bottom: 8px;
      font-weight: 600;
    }
    
    .status-bar {
      display: flex;
      justify-content: flex-start;
      flex-wrap: wrap;
      gap: 12px;
      font-size: 1.05rem;
    }
    
    .status-item {
      display: flex;
      align-items: center;
      background: rgba(255,255,255,0.15);
      padding: 8px 14px;
      border-radius: 18px;
    }
    
    .status-indicator {
      display: inline-block;
      width: 14px;
      height: 14px;
      border-radius: 50%;
      margin-right: 8px;
    }
    
    .status-online { background-color: #2ecc71; }
    .status-offline { background-color: #e74c3c; }
    
    .content {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      grid-template-areas:
        "control actions actions actions"
        "control status chartstack chartstack"
        "control status chartstack chartstack";
      grid-auto-rows: auto;
      align-items: stretch;
      gap: 20px;
      padding: 30px;
      flex: 1;
    }

    /* 兼容部分旧版华为/国产浏览器的栅格实现差异，强制在小屏或不支持grid时使用单列布局，避免组件挤在一起 */
    @media (max-width: 1100px) {
      .content {
        display: flex !important;
        flex-direction: column !important;
      }
    }

    @supports not (display: grid) {
      .content {
        display: flex !important;
        flex-direction: column !important;
      }
    }

    .actions-panel {
      grid-area: actions;
      display: flex;
      flex-direction: column;
      gap: 12px;
      padding: 18px;
      width: 99%;
    }

    .actions-row {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      justify-content: center;
    }

    .actions-row .action-btn {
      white-space: nowrap;
      min-width: 0;
    }

    .actions-row .main-action {
      flex: 0 0 180px;
    }

    .actions-row .preset-action {
      flex: 1 1 0;
    }

    .presets-row {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
      gap: 10px;
    }

    .control-panel { grid-area: control; }
    .status-panel { grid-area: status; }

    .right-stack {
      grid-area: chartstack;
      display: flex;
      flex-direction: column;
      gap: 18px;
    }
    
    @media (max-width: 768px) {
      header {
        padding: 14px 16px;
      }

      .header-inner {
        grid-template-columns: 1fr;
        justify-items: center;
        row-gap: 10px;
        text-align: center;
      }

      .header-title {
        flex-wrap: wrap;
        justify-content: center;
        text-align: center;
      }

      h1 {
        font-size: 1.8rem;
        margin-bottom: 4px;
      }

      .status-bar {
        flex-direction: column;
        align-items: center;
        width: 100%;
        gap: 8px;
      }
        
      .status-item {
        width: 100%;
        justify-content: center;
        text-align: center;
      }

      .config-btn {
        width: 140px;
        justify-content: center;
      }

        .chart-panel {
          margin: 15px;
          grid-column: 1;
        }

        .right-stack {
          grid-column: 1;
        }

        .actions-panel {
          grid-area: auto;
          width: 99%;
        }

        .actions-row {
          flex-wrap: wrap;
          justify-content: center;
        }

        .actions-row .main-action {
          flex: 1 1 100%;
        }

        .actions-row .preset-action {
          flex: 1 1 30%;
          min-width: 100px;
        }

        .temp-control {
          flex-direction: row;
          align-items: center;
          justify-content: center;
          gap: 12px;
        }

        .temp-display,
        .fan-display {
          align-items: center;
          text-align: center;
        }

        .container {
          padding: 4px;
          margin: 4px auto;
          min-height: calc(100vh - 8px);
        }
        
        .panel {
          padding: 16px;
          margin: 0 auto 12px;
          width: 100%;
        }

        .content {
          grid-template-columns: 1fr !important;
          display: flex !important;
          flex-direction: column !important;
        }

        .container {
            padding: 10px;
            margin: 10px auto;
        }
        
        .panel {
          padding: 15px;
          margin-bottom: 15px;
          width: 100%;
        }

        .control-panel,
        .status-panel,
        .right-stack {
          width: 100%;
        }
        
        .temp-btn,
        .time-btn {
          width: 48px;
          height: 48px;
          border-radius: 50%;
          padding: 0;
          font-size: 1rem;
          display: inline-flex;
          align-items: center;
          justify-content: center;
          min-width: 48px;
        }

        .temp-display,
        .fan-display {
          flex-direction: column;
          gap: 4px;
          align-items: center;
          text-align: center;
        }
        
        .temp-control,
        .time-control {
          flex-direction: row;
          align-items: center;
          justify-content: center;
          gap: 12px;
        }

        .fan-display {
          flex-direction: column;
          gap: 4px;
        }

        .time-display {
          margin: 10px 0 2px;
          font-size: 2rem;
          min-width: 120px;
        }

        .action-buttons {
            grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
            gap: 10px;
        }
        
        .action-btn {
            padding: 12px;
            font-size: 1rem;
        }
        
        .temp-presets {
            grid-template-columns: repeat(auto-fit, minmax(90px, 1fr));
        }
        
        .preset-btn {
            padding: 12px 8px;
            font-size: 1rem;
        }
        
        .status-row {
            flex-direction: column;
            align-items: flex-start;
            gap: 4px;
            padding: 10px 0;
        }

        .status-value {
            font-size: 1.2rem;
        }

        .status-bar {
            flex-direction: column;
            align-items: flex-start;
            gap: 8px;
        }
        
        .status-item {
            width: 100%;
            margin-bottom: 8px;
        }
    }

    @media (max-width: 520px) {
      .container {
        width: 100%;
        border-radius: 0;
      }

      .content {
        padding: 5px;
      }

      .actions-row {
        gap: 12px;
      }

      .actions-row .main-action {
        flex: 0 1 80%;
        max-width: 240px;
      }

      .actions-row .preset-action {
        flex: 1 1 45%;
        min-width: 110px;
        max-width: 150px;
      }

      .action-btn {
        padding: 8px 10px;
        font-size: 0.9rem;
        border-radius: 7px;
      }

      .preset-btn {
        padding: 8px 8px;
        font-size: 0.9rem;
        border-radius: 7px;
        margin-bottom: 4px;
      }

      .actions-panel,
      .panel {
        width: 100%;
        margin: 0 auto 10px;
        padding: 14px;
      }
    }
    
    .panel {
      background: #f8f9fa;
      border-radius: 12px;
      padding: 25px;
      box-shadow: 0 4px 10px rgba(0,0,0,0.05);
      border: 1px solid #e9ecef;
      height: 100%;
      display: flex;
      flex-direction: column;
    }
    
    .panel-title {
      font-size: 1.5rem;
      margin-bottom: 20px;
      padding-bottom: 12px;
      border-bottom: 2px solid #3498db;
      color: #2c3e50;
      display: flex;
      align-items: center;
    }
    
    .panel-title i {
      margin-right: 10px;
      color: #3498db;
    }
    
    .control-group {
      margin-bottom: 18px;
    }
    
    .control-label {
      display: block;
      font-size: 1.1rem;
      margin-bottom: 12px;
      font-weight: 500;
      color: #495057;
    }
    
    .temp-display {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 15px;
    }
    
    .temp-current {
      font-size: 2.8rem;
      font-weight: 700;
      color: #e74c3c;
    }
    
    .temp-target {
      font-size: 1.8rem;
      font-weight: 600;
      color: #3498db;
    }
    
    .temp-slider-container {
      margin: 12px 0 8px;
      position: relative;
      padding-top: 16px;
    }
    
    .temp-slider {
      width: 100%;
      height: 25px;
      -webkit-appearance: none;
      background: linear-gradient(to right, #3498db, #e74c3c);
      border-radius: 12px;
      outline: none;
    }
    
    .temp-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: white;
      border: 3px solid #2c3e50;
      cursor: pointer;
      box-shadow: 0 3px 10px rgba(0,0,0,0.2);
    }
    
    .temp-bubble {
      display: none;
    }
    
    .temp-bubble.show {
      opacity: 1;
    }

    .temp-display-updating {
      color: #e74c3c;
      transition: color 0.3s;
    }
    
    .time-display-updating {
      color: #e74c3c;
      transition: color 0.3s;
    }
    
    .fan-display-updating {
      color: #e74c3c;
      transition: color 0.3s;
    }

    .temp-bubble::after {
      content: '';
      position: absolute;
      bottom: -10px;
      left: 50%;
      transform: translateX(-50%);
      border-width: 10px 8px 0;
      border-style: solid;
      border-color: #2c3e50 transparent transparent;
    }

    .temp-presets {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
      gap: 12px;
      margin: 20px 0;
    }
    
    .preset-btn {
      padding: 15px 10px;
      background: #3498db;
      color: white;
      border: none;
      border-radius: 8px;
      font-size: 1.1rem;
      font-weight: 500;
      cursor: pointer;
      transition: all 0.3s ease;
      text-align: center;
    }
    
    .preset-btn:hover {
      background: #2980b9;
      transform: translateY(-3px);
      box-shadow: 0 5px 15px rgba(0,0,0,0.1);
    }
    
    .preset-btn.active {
      background: linear-gradient(135deg, #1abc9c 0%, #16a085 100%);
      border: 2px solid #0f7c6b;
      box-shadow: 0 6px 18px rgba(0,0,0,0.15), 0 0 0 3px rgba(26,188,156,0.28);
      transform: translateY(-2px) scale(1.03);
      color: #fff;
      font-weight: 700;
    }
    
    .time-control {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin: 20px 0;
    }
    
    .time-display {
      font-size: 2.5rem;
      font-weight: 700;
      min-width: 150px;
      text-align: center;
      color: #2c3e50;
      padding: 8px 15px;
      background-color: #f8f9fa;
    }
    
    .time-btn {
      width: 50px;
      height: 50px;
      border-radius: 50%;
      background: #3498db;
      color: white;
      font-size: 1.8rem;
      font-weight: bold;
      border: none;
      cursor: pointer;
      transition: all 0.3s;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 0;
      line-height: 1;
    }
    
    .time-btn:hover {
      background: #2980b9;
      transform: scale(1.1);
    }
    
    .action-buttons {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      gap: 15px;
      margin-top: 25px;
    }
    
    .action-btn {
      padding: 16px;
      border: none;
      border-radius: 8px;
      font-size: 1.2rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.3s;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
    }
    
    .btn-start {
      background: #2ecc71;
      color: white;
    }
    
    .btn-pause {
      background: #f39c12;
      color: white;
    }
    
    .btn-stop {
      background: #e74c3c;
      color: white;
    }
    
    .btn-pid {
      background: #9b59b6;
      color: white;
    }
    
    .btn-start:hover { background: #27ae60; transform: translateY(-3px); }
    .btn-pause:hover { background: #e67e22; transform: translateY(-3px); }
    .btn-stop:hover { background: #c0392b; transform: translateY(-3px); }
    .btn-pid:hover { background: #8e44ad; transform: translateY(-3px); }
    
    .btn-start:active, .btn-pause:active, 
    .btn-stop:active, .btn-pid:active {
      transform: translateY(0);
    }
    
    .status-display {
      background: #fff;
      border-radius: 10px;
      padding: 14px;
      margin-top: 12px;
      flex: 1;
    }
    
    .status-row {
      display: flex;
      justify-content: space-between;
      padding: 8px 0;
      border-bottom: 1px solid #eee;
      font-size: 1.1rem;
    }
    
    .status-label {
      font-weight: 500;
      color: #6c757d;
    }
    
    .status-value {
      font-weight: 600;
      color: #2c3e50;
    }
    
    .alert-box {
      background: #ffebee;
      border-left: 4px solid #e74c3c;
      padding: 15px;
      border-radius: 8px;
      margin-top: 20px;
      display: none;
    }
    
    .alert-title {
      font-weight: 600;
      color: #c0392b;
      margin-bottom: 8px;
    }
    
    .alert-message {
      font-size: 1.1rem;
      color: #e74c3c;
    }
    
    footer {
      text-align: center;
      padding: 20px;
      color: #6c757d;
      font-size: 0.9rem;
      border-top: 1px solid #eee;
    }
    
    .mqtt-link {
      display: inline-block;
      margin-top: 15px;
      padding: 12px 25px;
      background: #9b59b6;
      color: white;
      text-decoration: none;
      border-radius: 6px;
      font-weight: 500;
      transition: all 0.3s;
    }
    
    .mqtt-link:hover {
      background: #8e44ad;
      transform: translateY(-2px);
    }
    
    .btn-reset {
      background: #e74c3c;
      color: white;
    }
    
    .btn-reset:hover {
      background: #c0392b;
      transform: translateY(-3px);
    }
    
    .footer-buttons {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 15px;
      margin-top: 20px;
    }
    
    .footer-btn {
      padding: 12px 25px;
      background: #3498db;
      color: white;
      text-decoration: none;
      border-radius: 6px;
      font-weight: 500;
      transition: all 0.3s;
    }
    
    .footer-btn:hover {
      transform: translateY(-2px);
    }
    
    .reset-btn {
      background: #e74c3c;
    }
    
    .reset-btn:hover {
      background: #c0392b;
    }
    
    .modal {
      display: none;
      position: fixed;
      z-index: 1000;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0,0,0,0.6);
      align-items: center;
      justify-content: center;
      padding: 20px;
    }

    .modal[style*="block"], .modal[style*="flex"] {
      display: flex !important;
    }
    
    .modal-content {
      background-color: #fff;
      margin: 0 auto;
      padding: 25px;
      border-radius: 12px;
      box-shadow: 0 5px 25px rgba(0,0,0,0.3);
      width: 85%;
      max-width: 520px;
      max-height: 90vh;
      overflow-y: auto;
      text-align: center;
      padding-bottom: 36px;
    }
    
    .modal-title {
      font-size: 1.8rem;
      margin-bottom: 20px;
      color: #e74c3c;
    }
    
    .modal-message {
      font-size: 1.2rem;
      margin-bottom: 25px;
      color: #555;
    }
    
    .modal-buttons {
      display: flex;
      justify-content: center;
      gap: 20px;
      margin-top: 16px;
    }
    
    .modal-btn {
      padding: 12px 30px;
      border: none;
      border-radius: 6px;
      font-size: 1.1rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.3s;
      min-width: 120px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
    }
    
    .modal-confirm {
      background: #e74c3c;
      color: white;
    }
    
    .modal-confirm:hover {
      background: #c0392b;
    }
    
    .modal-cancel {
      background: #95a5a6;
      color: white;
    }
    
    .modal-cancel:hover {
      background: #7f8c8d;
    }

    .modal-pid {
      background: #9b59b6;
      color: white;
    }

    .modal-pid:hover {
      background: #8e44ad;
    }
    
    .loading-spinner {
      width: 40px;
      height: 40px;
      margin: 20px auto;
      border: 4px solid #f3f3f3;
      border-top: 4px solid #3498db;
      border-radius: 50%;
      animation: spin 1s linear infinite;
    }
    
    .chart-panel {
      margin: 0;
    }

    .config-btn {
      background: rgba(255,255,255,0.15);
      color: white;
      border: 1px solid rgba(255,255,255,0.3);
      padding: 10px 16px;
      border-radius: 10px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      line-height: 1.05;
      cursor: pointer;
      font-weight: 600;
      transition: all 0.2s ease;
      text-align: center;
    }

    .config-btn i {
      display: none;
    }

    .config-btn .btn-text {
      white-space: nowrap;
    }

    .config-btn .loading-spinner {
      display: none;
      margin: 0;
    }

    .config-btn.loading {
      opacity: 0.85;
      cursor: progress;
    }

    .config-btn.loading .loading-spinner {
      display: inline-block;
    }

    .config-btn.loading i {
      display: none;
    }

    .loading-spinner.small {
      width: 16px;
      height: 16px;
      border-width: 3px;
      margin: 0;
    }

    .config-btn:hover {
      background: rgba(255,255,255,0.25);
      transform: translateY(-1px);
    }

    .alert-panel .panel-title {
      border-color: #3498db;
    }

    .reset-box {
      background: #fff3cd;
      border: 1px solid #ffeeba;
      padding: 12px 14px;
      border-radius: 8px;
      margin-top: 14px;
      display: none;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      flex-wrap: wrap;
    }

    .reset-box strong { color: #856404; }

    .modal-settings {
      display: flex;
      flex-direction: column;
      gap: 14px;
      text-align: left;
    }

    .setting-block {
      border: 1px solid #e5eaf0;
      border-radius: 8px;
      padding: 12px 14px;
      background: #f8fbff;
    }

    .setting-block.plain {
      border: none;
      background: transparent;
      padding: 6px 0 4px 0;
    }

    .setting-row {
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 8px;
    }

    .modal-settings .status-row {
      border-bottom: none;
      padding: 6px 0;
    }

    .setting-row:first-child { margin-top: 0; }

    .setting-label {
      min-width: 180px;
      color: #555;
      font-weight: 600;
    }

    .setting-desc { color: #555; min-width: 150px; }

    .setting-row input[type="number"] { padding: 4px 6px; width: 80px; }

    .setting-actions { display: flex; justify-content: flex-end; }

    .switch {
      position: relative;
      display: inline-block;
      width: 48px;
      height: 26px;
    }

    .switch input { display:none; }

    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: #ccc;
      transition: .3s;
      border-radius: 26px;
    }

    .slider:before {
      position: absolute;
      content: "";
      height: 20px;
      width: 20px;
      left: 3px;
      bottom: 3px;
      background-color: white;
      transition: .3s;
      border-radius: 50%;
      box-shadow: 0 1px 4px rgba(0,0,0,0.2);
    }

    .switch input:checked + .slider {
      background-color: #2ecc71;
    }

    .switch input:checked + .slider:before {
      transform: translateX(22px);
    }

    .switch input:not(:checked) + .slider {
      background-color: #bdc3c7;
    }

    .config-actions {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
      margin: 16px 0;
    }

    .config-action {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 10px;
      padding: 12px;
      background: #3498db;
      color: white;
      border-radius: 8px;
      text-decoration: none;
      border: none;
      cursor: pointer;
      font-size: 1rem;
      font-weight: 600;
      transition: all 0.2s ease;
    }

    .config-action:hover {
      background: #2980b9;
      transform: translateY(-1px);
    }

    .config-action.danger {
      background: #e74c3c;
    }

    .config-action.danger:hover {
      background: #c0392b;
    }

    .chart-container {
      display: flex;
      flex-direction: column;
      align-items: center; /* 水平居中 */
      justify-content: center; /* 垂直居中 */
      width: 100%;
    }

    #tempChart {
      max-width: 100%;
      border: 1px solid #ddd; 
      background: #f9f9f9;
      display: block;
      margin: 0 auto; /* 水平居中 */
    }

    .chart-controls {
      margin-top: 10px; 
      text-align: center; /* 控制按钮居中 */
      width: 100%;
    }

    @keyframes spin {
      0% { transform: rotate(0deg); }
      100% { transform: rotate(360deg); }
    }
    
  </style>
</head>
<body>
  <div id="toast" class="toast"></div>
  <div class="container">
    <header>
      <div class="header-inner">
        <div class="status-bar">
          <div class="status-item">
            <span class="status-indicator status-online"></span>
            网络状态: 已连接
          </div>
          <div class="status-item">
            <span class="status-indicator status-online"></span>
            IP: )=====";
  html += WiFi.localIP().toString();
  html += R"=====(
          </div>
        </div>

        <div class="header-title">
          <h1>AMS-lite烘干器控制面板</h1>
        </div>

        <div class="header-right">
          <button class="config-btn" id="moreSettingsBtn"><span class="loading-spinner small" id="moreSettingsSpinner"></span><i class="fas fa-sliders-h"></i><span class="btn-text">更多烘干设置</span></button>
          <button class="config-btn" id="configBtn"><i class="fas fa-cog"></i> 配置</button>
        </div>
      </div>
    </header>
    
    <div class="content">
      <div class="panel actions-panel">
        <div class="actions-row">
          <button class="action-btn btn-start main-action" id="startBtn">
            <i class="fas fa-play"></i>开始烘干
          </button>
          <button class="action-btn btn-stop main-action" id="stopBtn">
            <i class="fas fa-stop"></i>停止烘干
          </button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 1 ? " active" : "");
        html += R"=====(" data-preset="1">PLA</button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 2 ? " active" : "");
        html += R"=====(" data-preset="2">PETG</button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 3 ? " active" : "");
        html += R"=====(" data-preset="3">ABS</button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 4 ? " active" : "");
        html += R"=====(" data-preset="4">ASA</button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 5 ? " active" : "");
        html += R"=====(" data-preset="5">PA</button>
          <button class="preset-btn preset-action)=====";
        html += (currentPreset == 0 ? " active" : "");
        html += R"=====(" data-preset="0">自定义</button>
        </div>
      </div>
      <div class="panel control-panel">
        <h2 class="panel-title"><i class="fas fa-sliders-h"></i> 烘干设置</h2>
        
        <div class="control-group">
          <label class="control-label">当前温度 / 目标温度</label>
          
          <div class="temp-control">
            <button class="temp-btn" id="tempMinusBtn">-5°</button>
            <div class="temp-display">
              <div class="temp-current">)=====";
  html += String(currentTemp, 1);
  html += R"=====(°C</div>
              <div class="temp-target">)=====";
  html += String(targetTemp, 0);
  html += R"=====(°C</div>
            </div>
            <button class="temp-btn" id="tempPlusBtn">+5°</button>
          </div>
          
          <div class="temp-slider-container">
            <input type="range" min="40" max="100" value=")=====";
  html += String(targetTemp, 0);
  html += R"=====(" class="temp-slider" id="tempSlider">
          </div>
        </div>

        <div class="control-group">
          <label class="control-label">烘干时间设置</label>
          
          <div class="temp-control">
            <button class="temp-btn" id="timeMinusBtn">-</button>
            <div class="temp-display">
              <div class="time-display" id="timeDisplay">)=====";
        html += String(targetTime);
        html += R"=====(分</div>
            </div>
            <button class="temp-btn" id="timePlusBtn">+</button>
          </div>
          
          <div class="temp-slider-container">
            <input type="range" min="0" max="1440" step="30" value=")=====";
        html += String(targetTime);
        html += R"=====(" class="temp-slider" id="timeSlider">
          </div>
        </div>

        <div class="control-group">
          <label class="control-label">风扇工作速度设置</label>
          <div class="temp-control">
            <button class="temp-btn" id="fanMinusBtn">-10%</button>
            <div class="fan-display" id="fanDisplay">50%</div>
            <button class="temp-btn" id="fanPlusBtn">+10%</button>
          </div>
          <div class="temp-slider-container">
            <input type="range" min=")=====";
  html += String(FAN_MIN_PWM);
  html += R"=====(" max="255" value="125" class="temp-slider" id="fanSlider">
          </div>
        </div>
      </div>
      
      <div class="panel status-panel">
        <h2 class="panel-title" style="display:flex;align-items:center;justify-content:space-between;gap:8px;">
          <span><i class="fas fa-info-circle"></i> 系统状态</span>
          <button id="performanceBtn" class="btn-small" style="padding:4px 10px;">esp32性能监控</button>
        </h2>
        
        <div class="status-display">
          <div class="status-row">
            <span class="status-label">设备状态:</span>
            <span class="status-value" id="stateValue">)=====";
  switch(currentState) {
    case STANDBY: html += "待机"; break;
    case DRYING: html += "运行中"; break;
  }
  html += R"=====(</span>
          </div>
          
          <div class="status-row">
            <span class="status-label">剩余时间(仅供参考):</span>
            <span class="status-value" id="remainingValue">--:--:--
            </span>
          </div>
          
          <div class="status-row">
            <span class="status-label">当前温度:</span>
            <span class="status-value" id="currentTempValue">)=====";
      html += String(currentTemp, 1);
      html += R"=====(℃</span>
          </div>

          <div class="status-row">
            <span class="status-label">设置温度:</span>
            <span class="status-value" id="targetTempValue">)=====";
  html += String(targetTemp, 0);
  html += R"=====(℃</span>
          </div>

          <div class="status-row">
            <span class="status-label">设置时间:</span>
            <span class="status-value" id="targetTimeValue">)=====";
  html += String(targetTime);
  html += R"=====(分钟</span>
          </div>
          
          <div class="status-row">
            <span class="status-label">当前模式:</span>
            <span class="status-value" id="modeValue">)=====";
  html += (currentPreset == 0) ? "自定义" : PRESETS[currentPreset-1].name;
  html += R"=====(</span>
          </div>

          <div class="status-row">
            <span class="status-label">仓温:</span>
            <span class="status-value" id="chamberTempValue">)=====";
  if (chamberSensorConnected) {
    html += String(chamberTemp, 1) + "°C";
  } else {
    html += "未连接仓温传感器";
  }
  html += R"=====(</span>
          </div>

          <div class="status-row">
            <span class="status-label">AHT30温度:</span>
            <span class="status-value" id="aht30TempValue">
              <span id="aht30TempDisplay">--</span>℃
            </span>
          </div>

          <div class="status-row">
            <span class="status-label">AHT30湿度:</span>
            <span class="status-value" id="aht30HumidityValue">
              <span id="aht30HumidityDisplay">--</span>%
            </span>
          </div>

          <div class="status-row">
            <span class="status-label">排气阀状态:</span>
            <span class="status-value" id="exhaustValveStatus">)=====";
html += getExhaustStateLabel();
html += R"=====(</span>
          </div>

          <div class="status-row">
            <span class="status-label">排气阀控制:</span>
            <span class="status-value" style="display:flex; gap:8px; flex-wrap:wrap;">
              <button class="btn-small" onclick="controlExhaust('open')">打开排气</button>
              <button class="btn-small" onclick="controlExhaust('close')">关闭排气</button>
            </span>
          </div>

        </div>
      </div>
      <div class="right-stack">
        <div class="panel chart-panel">
          <h2 class="panel-title"><i class="fas fa-chart-line"></i> 温度曲线</h2>
          <div class="chart-container">
            <canvas id="tempChart" width="1100" height="420"></canvas>
            <div class="chart-controls">
              <label style="margin-right: 15px;">
                <input type="checkbox" id="showMainTemp" checked> 主温度
              </label>
              <label>
                <input type="checkbox" id="showChamberTemp" checked> 仓温
              </label>
            </div>
          </div>
        </div>

        <div class="panel alert-panel">
          <h2 class="panel-title" style="display:flex;align-items:center;justify-content:space-between;gap:8px;">
            <span><i class="fas fa-bell"></i> 告警信息</span>
            <button id="clearAlertBtn" class="btn-small" style="padding:4px 10px;">清除</button>
          </h2>
          <div class="alert-box" id="alertBox" style="display:none;">
            <div class="alert-message" id="alertMessage">无告警信息</div>
          </div>
          <div class="reset-box" id="resetReasonBox">
            <div><strong>重启原因：</strong><span id="resetText">--</span></div>
            <button class="btn-small" id="clearReset">清除记录</button>
          </div>
        </div>
      </div>
    </div>
  </div>

    <div id="moreSettingsModal" class="modal">
      <div class="modal-content">
        <h2 class="modal-title">更多烘干设置</h2>
        <div class="modal-settings">
          <div class="setting-block">
            <div class="setting-row" style="justify-content: space-between; align-items:center;">
              <span class="status-label" style="margin:0;">PID整定温度(建议设置常用温度)</span>
              <div style="display:flex; align-items:center; gap:8px;">
                <input type="number" id="pidAutotuneTarget" min="40" max="120" step="0.5" style="width: 90px; padding:4px 6px;"> <span>°C</span>
                <button class="btn-small" onclick="savePidAutotuneTarget()">保存</button>
              </div>
            </div>
          </div>

          <div class="setting-block">
            <div class="status-row">
              <span class="status-label">排气阀模式</span>
              <span class="status-value">
                <label style="margin-right:10px;">
                  <input type="radio" name="exhaustMode" id="exhaustModeAuto" value="auto" checked> 自动
                </label>
                <label>
                  <input type="radio" name="exhaustMode" id="exhaustModeManual" value="manual"> 手动
                </label>
              </span>
            </div>
          </div>

          <div class="setting-block">
            <div class="status-row" style="align-items:center; gap:8px;">
              <span class="status-label">排气阀正反转设置</span>
              <span class="status-value" style="display:flex; align-items:center; gap:8px;">
                <span>正转</span>
                <label class="switch">
                  <input type="checkbox" id="exhaustDirectionSwitch" )=====";
  if (exhaustMotorReversed) html += "checked";
  html += R"=====(>
                  <span class="slider"></span>
                </label>
                <span>反转</span>
              </span>
            </div>
          </div>

          <div class="setting-block">
            <div class="status-row">
              <span class="status-label">排气阀动作时间</span>
              <span class="status-value">
                <input type="number" id="exhaustActionTime" min="3000" max="20000" step="1" 
                        value=")=====";
html += String(exhaustActionTime);
html += R"=====(" style="width: 80px; padding: 2px;"> ms
                <button class="btn-small" onclick="saveExhaustActionTime()">保存</button>
              </span>
            </div>
          </div>

          <div class="setting-block">
            <div class="setting-label">自动排气循环(分钟)</div>
            <div class="setting-row">
              <span class="setting-desc">开始烘干后首次开阀时间</span>
              <input type="number" id="exhaustFirstDelay" min="0" max="300" step="1" value=")=====";
  html += String(exhaustFirstDelay / 60000);
  html += R"=====("> <span>分钟</span>
            </div>
            <div class="setting-row">
              <span class="setting-desc">后续循环开启时间</span>
              <input type="number" id="exhaustOnDuration" min="1" max="300" step="1" value=")=====";
  html += String(exhaustOnDuration / 60000);
  html += R"=====("> <span>分钟</span>
            </div>
            <div class="setting-row">
              <span class="setting-desc">后续循环关闭时间</span>
              <input type="number" id="exhaustOffDuration" min="1" max="300" step="1" value=")=====";
  html += String(exhaustOffDuration / 60000);
  html += R"=====("> <span>分钟</span>
            </div>
            <div class="setting-actions">
              <button class="btn-small" onclick="saveExhaustCycle()">保存</button>
            </div>
          </div>

          <div class="setting-block">
            <div class="setting-row" style="justify-content: space-between;">
              <span class="status-label" style="margin:0;">自动烘干(湿度触发)</span>
              <label class="switch">
                <input type="checkbox" id="autoDrySwitch" )=====";
  if (autoDryEnabled) html += "checked";
  html += R"=====(>
                <span class="slider"></span>
              </label>
            </div>
            <div class="setting-row">
              <span class="setting-desc">开始湿度</span>
              <input type="number" id="autoDryStart" min="0" max="100" step="0.5" value=")=====";
  html += String(autoDryStartHumidity, 1);
  html += R"=====("> <span>%</span>
            </div>
            <div class="setting-row">
              <span class="setting-desc">运行时间</span>
              <input type="number" id="autoDryRun" min="1" max="1440" step="1" value=")=====";
  html += String(autoDryRunMinutes);
  html += R"=====("> <span>分钟</span>
            </div>
            <div class="setting-row">
              <span class="setting-desc">手动停止后的cd时间</span>
              <input type="number" id="autoDryCooldown" min="0" max="1440" step="1" value=")=====";
  html += String(autoDryCooldownMinutes);
  html += R"=====("> <span>分钟</span>
            </div>
            <div class="setting-actions">
              <button class="btn-small" onclick="saveAutoDry()">保存</button>
            </div>
          </div>
        </div>

        <div class="modal-buttons">
          <button class="modal-btn modal-pid" id="pidBtn"><i class="fas fa-cogs"></i> PID自动整定</button>
          <button class="modal-btn modal-cancel" id="closeMoreSettings">关闭</button>
        </div>
      </div>
    </div>

    <div id="configModal" class="modal">
      <div class="modal-content">
        <h2 class="modal-title">配置</h2>
        <div class="config-actions">
          <a href="/mqtt-config" class="config-action"><i class="fas fa-server"></i> 配置MQTT服务器</a>
          <a href="/wifi-config" class="config-action"><i class="fas fa-wifi"></i> 配置WiFi网络</a>
          <a href="/firmware-update" class="config-action"><i class="fas fa-upload"></i> 固件更新</a>
          <button class="config-action danger" id="resetBtn"><i class="fas fa-undo"></i> 恢复出厂设置</button>
        </div>
        <button class="modal-btn modal-cancel" id="closeConfig">关闭</button>
      </div>
    </div>

    <div id="factoryResetModal" class="modal">
    <div class="modal-content">
      <h2 class="modal-title">确认恢复出厂设置</h2>
      <p class="modal-message">此操作将清除所有配置，但不会回退固件版本。</p>
      <p class="modal-message">（包括网络、PID参数、风速、等等）</p>
      <p class="modal-message">设备会在执行后自动重启。</p>
      <p class="modal-message">您确定要继续吗？</p>
      
      <div class="modal-buttons">
        <button class="modal-btn modal-cancel" id="cancelReset">取消</button>
        <button class="modal-btn modal-confirm" id="confirmReset">确认重置</button>
      </div>
    </div>
  </div>

  <div id="performanceModal" class="modal">
    <div class="modal-content">
      <h2 class="modal-title" style="display:flex;align-items:center;justify-content:space-between;gap:8px;">
        <span>ESP32性能监控</span>
        <button class="modal-btn modal-cancel" id="closePerformance" style="padding:8px 16px;">关闭</button>
      </h2>
      <div class="modal-settings">
        <div class="setting-block">
          <div class="status-row" style="flex-direction: column; align-items: flex-start; gap: 12px; padding: 15px 0;">
            <div style="width: 100%; display: flex; flex-direction: column; gap: 8px;">
              <div style="display: flex; justify-content: space-between; align-items: center; width: 100%;">
                <span class="status-label">CPU使用率</span>
                <span class="status-value" id="cpuUsage">0%</span>
              </div>
              <div style="width: 100%; height: 15px; background-color: #f0f0f0; border-radius: 8px; overflow: hidden;">
                <div id="cpuProgress" style="height: 100%; background-color: #2ecc71; width: 0%; transition: width 0.3s, background-color 0.3s;"></div>
              </div>
            </div>

            <div style="width: 100%; display: flex; flex-direction: column; gap: 8px;">
              <div style="display: flex; justify-content: space-between; align-items: center; width: 100%;">
                <span class="status-label">RAM (内置)</span>
                <span class="status-value" id="ramUsage">0% (0.00MB/0.33MB)</span>
              </div>
              <div style="width: 100%; height: 15px; background-color: #f0f0f0; border-radius: 8px; overflow: hidden;">
                <div id="ramProgress" style="height: 100%; background-color: #2ecc71; width: 0%; transition: width 0.3s, background-color 0.3s;"></div>
              </div>
            </div>

            <div style="width: 100%; display: flex; flex-direction: column; gap: 8px;">
              <div style="display: flex; justify-content: space-between; align-items: center; width: 100%;">
                <span class="status-label">PSRAM (R8)</span>
                <span class="status-value" id="psramUsage">0% (0.00MB/8MB)</span>
              </div>
              <div style="width: 100%; height: 15px; background-color: #f0f0f0; border-radius: 8px; overflow: hidden;">
                <div id="psramProgress" style="height: 100%; background-color: #2ecc71; width: 0%; transition: width 0.3s, background-color 0.3s;"></div>
              </div>
            </div>

            <div style="width: 100%; display: flex; flex-direction: column; gap: 8px;">
              <div style="display: flex; justify-content: space-between; align-items: center; width: 100%;">
                <span class="status-label">Flash (N16)</span><span class="status-hint" style="font-size: 10px; color: #666; margin-left: 5px;">(部分容量被系统和更新分区占用)</span>
                <span class="status-value" id="flashUsage">0% (0.00MB/16MB)</span>
              </div>
              <div style="width: 100%; height: 15px; background-color: #f0f0f0; border-radius: 8px; overflow: hidden;">
                <div id="flashProgress" style="height: 100%; background-color: #2ecc71; width: 0%; transition: width 0.3s, background-color 0.3s;"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    // 全局状态变量
    let isStandby = true; // 默认为待机状态
    let remainingTimeSeconds = 0; // 存储剩余时间（秒）
    let lastUpdateTime = 0; // 上次状态更新时间
    let countdownInterval = null; // 倒计时定时器
    let lastState = ''; // 存储上次状态
    let localTargetTemp = 0;    // 本地缓存的温度值
    let isAdjustingTemp = false; // 是否正在调整温度
    let tempDebounceTimer = null; // 温度调整防抖计时器
    const DEBOUNCE_DELAY = 500; // 防抖延迟
    let localTargetTime = 0;    // 本地缓存的时间值
    let isAdjustingTime = false; // 是否正在调整时间
    let timeDebounceTimer = null; // 时间调整防抖计时器
)=====";
  html += "    const deviceApSsid = \"" + apSSID + "\";\n";
  html += "    const deviceApIp = \"192.168.4.1\";\n";
  html += R"=====(
    const fanSlider = document.getElementById('fanSlider');
    const fanDisplay = document.getElementById('fanDisplay');
    const fanMinusBtn = document.getElementById('fanMinusBtn');
    const fanPlusBtn = document.getElementById('fanPlusBtn');
    let fanDebounceTimer = null;
    let isAdjustingFan = false;
    const exhaustModeAuto = document.getElementById('exhaustModeAuto');
    const exhaustModeManual = document.getElementById('exhaustModeManual');
    const exhaustDirectionSwitch = document.getElementById('exhaustDirectionSwitch');
    const autoDrySwitch = document.getElementById('autoDrySwitch');
    let isMoreSettingsOpen = false;
    let lastAutotuneResultKey = localStorage.getItem('lastAutotuneResultKey') || '';

    function formatExhaustState(stateCode, fallbackOpen) {
      switch (stateCode) {
        case 'opening': return '开启中';
        case 'closing': return '关闭中';
        case 'open': return '打开';
        case 'closed': return '关闭';
        default: return fallbackOpen ? '打开' : '关闭';
      }
    }

    [exhaustModeAuto, exhaustModeManual].forEach(radio => {
      if (radio) {
        radio.addEventListener('change', () => {
          if (radio.checked) {
            setExhaustMode(radio.value);
          }
        });
      }
    });

    if (autoDrySwitch) {
      autoDrySwitch.addEventListener('change', () => {
        toggleAutoDry(autoDrySwitch.checked);
      });
    }
    if (exhaustDirectionSwitch) {
      exhaustDirectionSwitch.addEventListener('change', () => {
        setExhaustDirection(exhaustDirectionSwitch.checked);
      });
    }
    // 温度图表相关变量
    let tempChartCanvas = document.getElementById('tempChart');
    let tempChartCtx = tempChartCanvas.getContext('2d');
    let temperatureHistory = [];
    let chartWidth = tempChartCanvas.width;
    let chartHeight = tempChartCanvas.height;
    let padding = 50;
    let showMainTemp = true;
    let showChamberTemp = true;
    let baseTime = 0;

    // 排气阀控制函数
    function controlExhaust(action) {
      fetch('/control?cmd=exhaust&action=' + action)
        .then(response => response.text())
        .then(data => {
          showToast('排气阀操作已发送: ' + action);
          updateStatus();
        })
        .catch(error => {
          console.error('排气阀操作失败:', error);
          showToast('操作失败');
        });
    }

    function setExhaustDirection(isReversed) {
      const value = isReversed ? 'reverse' : 'forward';
      fetch('/control?cmd=exhaust&action=set_direction&value=' + value)
        .then(response => response.text())
        .then(data => {
          showToast('电机方向已设置为' + (isReversed ? '反转' : '正转'));
          updateStatus();
        })
        .catch(error => {
          console.error('设置方向失败:', error);
          showToast('操作失败');
        });
    }

    function savePidAutotuneTarget() {
      const input = document.getElementById('pidAutotuneTarget');
      const val = parseFloat(input.value);
      if (isNaN(val)) {
        showToast('请输入有效的温度');
        return;
      }
      fetch(`/control?cmd=pid_autotune&action=set_target&value=${val}`)
        .then(response => response.text())
        .then(text => {
          if (text === 'target_updated') {
            showToast('PID整定温度已保存');
            updateStatus();
          } else {
            showToast('温度超出范围');
          }
        })
        .catch(err => {
          console.error('保存PID整定温度失败', err);
          showToast('保存失败');
        });
    }

    function saveExhaustActionTime() {
      const timeInput = document.getElementById('exhaustActionTime');
      const timeMs = parseInt(timeInput.value, 10);
      
      if (timeMs >= 3000 && timeMs <= 20000) {
        fetch('/control?cmd=exhaust&action=set_time&value=' + timeMs)
          .then(response => response.text())
          .then(data => {
            showToast('排气阀动作时间已保存: ' + timeMs + ' ms');
            updateStatus();
          })
          .catch(error => {
            console.error('保存时间失败:', error);
            showToast('保存失败');
          });
      } else {
        showToast('请输入3000-20000之间的毫秒数');
      }
    }

    function saveExhaustCycle() {
      const first = parseInt(document.getElementById('exhaustFirstDelay').value, 10);
      const on = parseInt(document.getElementById('exhaustOnDuration').value, 10);
      const off = parseInt(document.getElementById('exhaustOffDuration').value, 10);

      const isValid = ![first, on, off].some(v => isNaN(v)) && first >= 0 && on >= 1 && off >= 1 && first <= 300 && on <= 300 && off <= 300;
      if (!isValid) {
        showToast('请输入0-300分钟的延迟，1-300分钟的开/关时间');
        return;
      }

      const url = `/control?cmd=exhaust&action=set_cycle&first=${first}&on=${on}&off=${off}`;
      fetch(url)
        .then(response => response.text())
        .then(data => {
          showToast('排气自动循环已保存');
          updateStatus();
        })
        .catch(error => {
          console.error('保存排气自动循环失败:', error);
          showToast('保存失败');
        });
    }

    function toggleAutoDry(enabled) {
      fetch(`/control?cmd=auto_dry&action=switch&value=${enabled ? 'on' : 'off'}`)
        .then(response => response.text())
        .then(data => {
          showToast(`自动烘干已${enabled ? '开启' : '关闭'}`);
          updateStatus();
        })
        .catch(error => {
          console.error('切换自动烘干失败:', error);
          showToast('切换失败');
          const sw = document.getElementById('autoDrySwitch');
          if (sw) sw.checked = !enabled;
        });
    }

    // 为输入框绑定回车触发保存
    function bindEnterSave(inputId, handler) {
      const el = document.getElementById(inputId);
      if (!el || typeof handler !== 'function') return;
      el.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
          e.preventDefault();
          handler();
        }
      });
    }

    bindEnterSave('pidAutotuneTarget', savePidAutotuneTarget);
    bindEnterSave('exhaustActionTime', saveExhaustActionTime);
    bindEnterSave('exhaustFirstDelay', saveExhaustCycle);
    bindEnterSave('exhaustOnDuration', saveExhaustCycle);
    bindEnterSave('exhaustOffDuration', saveExhaustCycle);
    bindEnterSave('autoDryStart', saveAutoDry);
    bindEnterSave('autoDryRun', saveAutoDry);
    bindEnterSave('autoDryCooldown', saveAutoDry);

    function saveAutoDry() {
      const startHum = parseFloat(document.getElementById('autoDryStart').value);
      const runMin = parseInt(document.getElementById('autoDryRun').value, 10);
      const cooldownMin = parseInt(document.getElementById('autoDryCooldown').value, 10);
      const sw = document.getElementById('autoDrySwitch');
      const enabled = sw ? sw.checked : false;

      const valid = !isNaN(startHum) && startHum >= 0 && startHum <= 100 &&
                    !isNaN(runMin) && runMin > 0 &&
                    !isNaN(cooldownMin) && cooldownMin >= 0;
      if (!valid) {
        showToast('请输入0-100的开始湿度，运行时间需大于0分钟，禁用时长需大于等于0分钟');
        return;
      }

      const url = `/control?cmd=auto_dry&action=threshold&start=${startHum}&duration=${runMin}&cooldown=${cooldownMin}`;
      fetch(url)
        .then(response => response.text())
        .then(data => {
          showToast('自动烘干参数已保存');
          // 切换状态(保持当前开关状态)
          return fetch(`/control?cmd=auto_dry&action=switch&value=${enabled ? 'on' : 'off'}`);
        })
        .then(() => {
          updateStatus();
        })
        .catch(error => {
          console.error('保存自动烘干配置失败:', error);
          showToast('保存失败');
        });
    }

    function setExhaustMode(mode) {
      fetch('/control?cmd=exhaust&action=set_mode&value=' + mode)
        .then(response => response.text())
        .then(data => {
          showToast('排气阀模式已切换为: ' + (mode === 'auto' ? '自动' : '手动'));
          updateStatus();
        })
        .catch(error => {
          console.error('切换排气模式失败:', error);
          showToast('切换失败');
        });
    }

    // 初始化图表控制
    document.getElementById('showMainTemp').addEventListener('change', function() {
      showMainTemp = this.checked;
      drawTemperatureChart();
    });
    
    document.getElementById('showChamberTemp').addEventListener('change', function() {
      showChamberTemp = this.checked;
      drawTemperatureChart();
    });

    // 绘制温度图表
    function drawTemperatureChart() {
      // 清除画布
      tempChartCtx.clearRect(0, 0, chartWidth, chartHeight);
      
      if (temperatureHistory.length === 0) {
        // 没有数据时显示提示
        tempChartCtx.fillStyle = '#999';
        tempChartCtx.font = '14px Arial';
        tempChartCtx.textAlign = 'center';
        tempChartCtx.fillText('暂无温度数据', chartWidth / 2, chartHeight / 2);
        return;
      }
      
      // 设置图表参数
      const graphWidth = chartWidth - padding * 2;
      const graphHeight = chartHeight - padding * 2;
      const minTemp = 0;
      const maxTemp = 120;
      const tempRange = maxTemp - minTemp;

      // 绘制坐标轴
      tempChartCtx.strokeStyle = '#ccc';
      tempChartCtx.lineWidth = 1;
      tempChartCtx.beginPath();
      tempChartCtx.moveTo(padding, padding);
      tempChartCtx.lineTo(padding, chartHeight - padding);
      tempChartCtx.lineTo(chartWidth - padding, chartHeight - padding);
      tempChartCtx.stroke();
      
      // 绘制设定温度线,只在烘干中或PID整定时显示
      if (!isStandby) {
        const targetTempY = chartHeight - padding - (graphHeight * (localTargetTemp - minTemp) / tempRange);
        
        // 确保Y坐标在图表范围内
        if (targetTempY >= padding && targetTempY <= chartHeight - padding) {
          // 绘制灰色虚线
          tempChartCtx.strokeStyle = '#7a7a7aff';
          tempChartCtx.lineWidth = 1;
          tempChartCtx.setLineDash([5, 3]); // 虚线样式
          tempChartCtx.beginPath();
          tempChartCtx.moveTo(padding, targetTempY);
          tempChartCtx.lineTo(chartWidth - padding, targetTempY);
          tempChartCtx.stroke();
          tempChartCtx.setLineDash([]); // 恢复实线
          
          // 绘制温度标签
          tempChartCtx.fillStyle = '#666';
          tempChartCtx.font = '14px Arial';
          tempChartCtx.textAlign = 'left';
          tempChartCtx.fillText(localTargetTemp + '°C', chartWidth - padding + 4, targetTempY + 5);
        }
      }

      // 绘制网格和标签
      tempChartCtx.strokeStyle = '#eee';
      tempChartCtx.fillStyle = '#666';
      tempChartCtx.font = '14px Arial';
      tempChartCtx.textAlign = 'right';
      
      // Y轴标签（温度）
      const yTicks = [0, 20, 40, 60, 80, 100, 120];
      yTicks.forEach(temp => {
        const y = chartHeight - padding - (graphHeight * (temp - minTemp) / tempRange);
        
        tempChartCtx.beginPath();
        tempChartCtx.moveTo(padding - 5, y);
        tempChartCtx.lineTo(padding, y);
        tempChartCtx.stroke();
        
        const labelText = temp + '°C';
        tempChartCtx.textAlign = 'right';
        tempChartCtx.fillText(labelText, padding - 8, y + 3);
      });
      
      // X轴标签（时间）
      tempChartCtx.textAlign = 'center';
      const timeRange = 5 * 60 * 1000; // 5分钟
      
      for (let i = 0; i <= 5; i++) {
        const x = padding + (graphWidth * i / 5);
        const timeOffset = (timeRange * i / 5);
        const minutes = Math.floor(timeOffset / 60000);
        
        tempChartCtx.beginPath();
        tempChartCtx.moveTo(x, chartHeight - padding);
        tempChartCtx.lineTo(x, chartHeight - padding + 5);
        tempChartCtx.stroke();
        
        if (i === 0) {
          tempChartCtx.fillText('前5分', x, chartHeight - padding + 15);
        } else if (i === 5) {
          tempChartCtx.fillText('当前', x, chartHeight - padding + 15);
        } else {
          tempChartCtx.fillText('前' + (5 - minutes) + '分', x, chartHeight - padding + 15);
        }
      }
      
      // 绘制温度曲线
      if (showMainTemp) {
        drawTemperatureLine('mainTemp', '#e74c3c', '主温度');
      }
      
      if (showChamberTemp) {
        drawTemperatureLine('chamberTemp', '#3498db', '仓温');
      }

      drawFixedLegend();
    }

    function drawFixedLegend() {
      const legendX = chartWidth - 100;
      const legendY = padding + 20;
      
      // 主温度图例
      if (showMainTemp) {
        tempChartCtx.fillStyle = '#e74c3c';
        tempChartCtx.fillRect(legendX, legendY, 15, 10);
        tempChartCtx.fillStyle = '#333';
        tempChartCtx.font = '14px Arial';
        tempChartCtx.textAlign = 'left';
        tempChartCtx.fillText('主温度', legendX + 20, legendY + 8);
      }
      
      // 仓温图例
      if (showChamberTemp) {
        tempChartCtx.fillStyle = '#3498db';
        tempChartCtx.fillRect(legendX, legendY + 20, 15, 10);
        tempChartCtx.fillStyle = '#333';
        tempChartCtx.font = '14px Arial';
        tempChartCtx.textAlign = 'left';
        tempChartCtx.fillText('仓温', legendX + 20, legendY + 28);
      }
    }

    function drawTemperatureLine(type, color, label) {
        const graphWidth = chartWidth - padding * 2;
        const graphHeight = chartHeight - padding * 2;
        const minTemp = 0;
        const maxTemp = 120;
        const tempRange = maxTemp - minTemp;
        
        // 绘制曲线
        tempChartCtx.strokeStyle = color;
        tempChartCtx.lineWidth = 2;
        tempChartCtx.beginPath();
        
        let firstPoint = true;
        const timeRange = 5 * 60 * 1000; // 5分钟
        
        // 按时间排序，确保正确的绘制顺序
        const sortedHistory = [...temperatureHistory].sort((a, b) => a.time - b.time);
        
        sortedHistory.forEach((point, index) => {
            const temp = point[type];
            // 只绘制有效温度点
            if (temp === null || temp === undefined || temp <= 0) {console.log('点',index,'温度无效,temp=',temp);return;}
            
            const timeOffset = baseTime - point.time;
            if (timeOffset > timeRange) {console.log('时间超过5分钟');return;}
            
            const x = chartWidth - padding - (graphWidth * timeOffset / timeRange);
            const y = chartHeight - padding - (graphHeight * (temp - minTemp) / tempRange);

            // 确保坐标在画布范围内
            if (x < padding || x > chartWidth - padding || y < padding || y > chartHeight - padding) {console.log('点坐标不在画布范围内');return;}
            
            if (firstPoint) {
                tempChartCtx.moveTo(x, y);
                firstPoint = false;
            } else {
                tempChartCtx.lineTo(x, y);
            }
        });
        
        tempChartCtx.stroke();
    }

    // 显示toast提示
    function showToast(message) {
      const toast = document.getElementById('toast');
      toast.textContent = message;
      toast.classList.add('show');
      
      setTimeout(() => {
        toast.classList.remove('show');
      }, 3000);
    }
    
    // 更新按钮状态
    function updateButtonState() {
      // 获取所有控制元素
      const tempMinusBtn = document.getElementById('tempMinusBtn');
      const tempPlusBtn = document.getElementById('tempPlusBtn');
      const tempSlider = document.getElementById('tempSlider');
      const timeMinusBtn = document.getElementById('timeMinusBtn');
      const timePlusBtn = document.getElementById('timePlusBtn');
      const presetButtons = document.querySelectorAll('.preset-btn');
      const startBtn = document.getElementById('startBtn');
      const pidBtn = document.getElementById('pidBtn');
      
      // 根据是否待机状态设置按钮状态
      if (isStandby) {
        // 待机状态 - 启用所有按钮
        tempMinusBtn.classList.remove('btn-disabled');
        tempPlusBtn.classList.remove('btn-disabled');
        tempSlider.removeAttribute('disabled');
        timeMinusBtn.classList.remove('btn-disabled');
        timePlusBtn.classList.remove('btn-disabled');
        timeSlider.removeAttribute('disabled');
        presetButtons.forEach(btn => btn.classList.remove('disabled'));
        startBtn.classList.remove('btn-disabled');
        pidBtn.classList.remove('btn-disabled');
      } else {
        // 非待机状态 - 禁用控制按钮
        tempMinusBtn.classList.add('btn-disabled');
        tempPlusBtn.classList.add('btn-disabled');
        tempSlider.setAttribute('disabled', true);
        timeMinusBtn.classList.add('btn-disabled');
        timePlusBtn.classList.add('btn-disabled');
        timeSlider.setAttribute('disabled', true);
        presetButtons.forEach(btn => btn.classList.add('disabled'));
        startBtn.classList.add('btn-disabled');
        pidBtn.classList.add('btn-disabled');
      }
    }

    // 初始化本地缓存值
    localTargetTemp = parseInt(document.querySelector('.temp-target').textContent);
    localTargetTime = parseInt(document.getElementById('timeDisplay').textContent);

    // 更新页面元素
    const stateValue = document.getElementById('stateValue');
    const remainingValue = document.getElementById('remainingValue');
    const alertBox = document.getElementById('alertBox');
    const alertMessage = document.getElementById('alertMessage');
    const resetBox = document.getElementById('resetReasonBox');
    const resetText = document.getElementById('resetText');
    const clearResetBtn = document.getElementById('clearReset');
    const clearAlertBtn = document.getElementById('clearAlertBtn');

    if (clearResetBtn && resetBox) {
      clearResetBtn.addEventListener('click', () => {
        fetch('/clear-reset', { method: 'POST' }).then(() => {
          resetBox.style.display = 'none';
        });
      });
    }

    if (clearAlertBtn && alertBox) {
      clearAlertBtn.addEventListener('click', () => {
        fetch('/clear-alert', { method: 'POST' })
          .then(() => {
            alertBox.style.display = 'none';
            alertMessage.textContent = '无告警信息';
          })
          .catch(err => console.error('清除告警失败:', err));
      });
    }

    // 计算风扇百分比
    function calculateFanPercent(speed) {
      return Math.round((speed / 255) * 100);
    }

    // 更新风扇UI
    function updateFanUI(speed) {
      const percent = calculateFanPercent(speed);
      fanSlider.value = speed;
      fanDisplay.textContent = percent + '%';
    }

    // 风扇滑块输入事件
    fanSlider.addEventListener('input', function() {
      if (!isStandby) return;

      const value = parseInt(this.value);
      const percent = calculateFanPercent(value);
      
      // 直接更新上方显示
      fanDisplay.textContent = percent + '%';
      fanDisplay.classList.add('fan-display-updating');
      
      // 更新本地状态
      isAdjustingFan = true;
    });

    // 风扇滑块值变化事件
    fanSlider.addEventListener('change', function() {
      if (!isStandby) return;

      const value = parseInt(this.value);
      const percent = calculateFanPercent(value);
      
      // 确保显示一致
      fanDisplay.textContent = percent + '%';
      
      // 设置防抖计时器
      clearTimeout(fanDebounceTimer);
      fanDebounceTimer = setTimeout(() => {
        // 移除更新样式
        fanDisplay.classList.remove('fan-display-updating');
        
        // 发送风扇设置命令
        setFanSpeed(value);
        isAdjustingFan = false;
      }, DEBOUNCE_DELAY);
    });

    // 风扇减少按钮
    fanMinusBtn.addEventListener('click', function() {
      let newSpeed = parseInt(fanSlider.value) - 25;
      newSpeed = Math.max(0, newSpeed);
      updateFanUI(newSpeed);
      setFanSpeed(newSpeed);
    });

    // 风扇增加按钮
    fanPlusBtn.addEventListener('click', function() {
      let newSpeed = parseInt(fanSlider.value) + 25;
      newSpeed = Math.min(255, newSpeed);
      updateFanUI(newSpeed);
      setFanSpeed(newSpeed);
    });
    
    // === 温度调整函数 ===
    function adjustTemperature(change) {
      if (!isStandby) {
        showToast("请结束当前任务后再修改");
        return;
      }

      // 取消之前的防抖计时器
      clearTimeout(tempDebounceTimer);
      
      // 更新本地缓存值（在有效范围内）
      let newTemp = localTargetTemp + change;
      newTemp = Math.max(40, Math.min(100, newTemp));
      newTemp = Math.round(newTemp / 5) * 5;
      
      localTargetTemp = newTemp;
      isAdjustingTemp = true;
      
      // 更新本地显示
      document.querySelector('.temp-target').textContent = newTemp + '°C';
      document.getElementById('tempSlider').value = newTemp;
      
      // 设置防抖计时器
      tempDebounceTimer = setTimeout(() => {
      // 发送到设备
      setTemperature(newTemp);
      isAdjustingTemp = false;
      }, DEBOUNCE_DELAY);
    }

    // 温度滑块
    const tempSlider = document.getElementById('tempSlider');
    const tempTargetDisplay = document.querySelector('.temp-target');
    
    // 温度滑块输入事件
    tempSlider.addEventListener('input', function() {
      if (!isStandby) return;
      // 读取滑块值并调整为5度步长
      let newTemp = parseInt(this.value);
      newTemp = Math.max(40, Math.min(100, newTemp));
      newTemp = Math.round(newTemp / 5) * 5;
      
      // 直接更新上方显示
      tempTargetDisplay.textContent = newTemp + '°C';
      tempTargetDisplay.classList.add('temp-display-updating');
      
      // 更新本地缓存
      localTargetTemp = newTemp;
      isAdjustingTemp = true;
    });
    
    // 温度滑块值变化事件
    tempSlider.addEventListener('change', function() {
        if (!isStandby) return;
        // 清除之前的防抖计时器
        clearTimeout(tempDebounceTimer);
        
        // 读取滑块值并调整为5度步长
        let newTemp = parseInt(this.value);
        newTemp = Math.max(40, Math.min(100, newTemp));
        newTemp = Math.round(newTemp / 5) * 5;
        
        // 确保显示一致
        this.value = newTemp;
        tempTargetDisplay.textContent = newTemp + '°C';
        
        // 设置防抖计时器
      tempDebounceTimer = setTimeout(() => {
        // 移除更新样式
        tempTargetDisplay.classList.remove('temp-display-updating');
        
        // 发送到设备
        setTemperature(newTemp);
        isAdjustingTemp = false;
      }, DEBOUNCE_DELAY);
    });
    
    // 温度增加按钮
    document.getElementById('tempPlusBtn').addEventListener('click', function() {
      adjustTemperature(5);
    });
    
    // 温度减少按钮
    document.getElementById('tempMinusBtn').addEventListener('click', function() {
      adjustTemperature(-5);
    });
    
    // 时间滑块功能
    const timeSlider = document.getElementById('timeSlider');
    const timeDisplay = document.getElementById('timeDisplay');

    // 时间滑块输入事件
    timeSlider.addEventListener('input', function() {
      if (!isStandby) return;
      // 读取滑块值并调整为30分钟步长
      let newTime = parseInt(this.value);
      newTime = Math.max(0, Math.min(1440, newTime));
      newTime = Math.round(newTime / 30) * 30;
      
      // 直接更新上方显示
      timeDisplay.textContent = newTime + '分';
      timeDisplay.classList.add('time-display-updating');
      
      // 更新本地缓存
      localTargetTime = newTime;
      isAdjustingTime = true;
    });

    // 时间滑块值变化事件
    timeSlider.addEventListener('change', function() {
      if (!isStandby) return;
      // 清除之前的防抖计时器
      clearTimeout(timeDebounceTimer);
      
      // 读取滑块值并调整为30分钟步长
      let newTime = parseInt(this.value);
      newTime = Math.max(0, Math.min(1440, newTime));
      newTime = Math.round(newTime / 30) * 30;
      
      // 确保显示一致
      this.value = newTime;
      timeDisplay.textContent = newTime + '分';
      
      // 设置防抖计时器
      timeDebounceTimer = setTimeout(() => {
        // 移除更新样式
        timeDisplay.classList.remove('time-display-updating');
        
        // 发送到设备
        setTime(newTime);
        isAdjustingTime = false;
      }, DEBOUNCE_DELAY);
    });

    // 时间减少按钮
    document.getElementById('timeMinusBtn').addEventListener('click', function() {
      adjustTime(-30);
    });

    // 时间增加按钮
    document.getElementById('timePlusBtn').addEventListener('click', function() {
      adjustTime(30);
    });

    // 时间调整函数
    function adjustTime(change) {
      if (!isStandby) {
        showToast("请结束当前任务后再修改");
        return;
      }

      // 取消之前的防抖计时器
      clearTimeout(timeDebounceTimer);
      
      // 更新本地缓存值（在有效范围内）
      let newTime = localTargetTime + change;
      newTime = Math.max(0, Math.min(1440, newTime));
      newTime = Math.round(newTime / 30) * 30;
      
      localTargetTime = newTime;
      isAdjustingTime = true;
      
      // 更新本地显示
      document.getElementById('timeDisplay').textContent = newTime + "分";
      document.getElementById('timeSlider').value = newTime;
      
      // 设置防抖计时器
      timeDebounceTimer = setTimeout(() => {
        // 发送到设备
        setTime(newTime);
        isAdjustingTime = false;
      }, DEBOUNCE_DELAY);
    }

    // 预设方案
    document.querySelectorAll('.preset-btn').forEach(btn => {
      btn.addEventListener('click', function() {
        if (!isStandby) {
          showToast("请结束当前任务后再修改");
          return;
        }
        setPreset(this.dataset.preset);
      });
    });
    
    // 控制按钮
    document.getElementById('startBtn').addEventListener('click', function() {
      if (!isStandby) {
        showToast("请结束当前任务后再修改");
        return;
      }
      performOperation('/control?cmd=start');
    });
       
    document.getElementById('stopBtn').addEventListener('click', function() {
      performOperation('/control?cmd=stop');
    });
    
    document.getElementById('pidBtn').addEventListener('click', function() {
      if (!isStandby) {
        showToast("请结束当前任务后再修改");
        return;
      }
      startPID();
    });
    
    // API调用函数
    async function setTemperature(temp) {
      await performOperation('/control?cmd=settemp&value=' + temp);
    }
       
    async function setTime(time) {
      await performOperation('/control?cmd=settime&value=' + time);
    }

    async function setPreset(preset) {
      await performOperation('/control?cmd=setpreset&value=' + preset);
    }
    
    async function setFanSpeed(speed) {
      await performOperation('/control?cmd=setfan&value=' + speed);
    }

    async function startDrying() {
      await performOperation('/control?cmd=start');
    }
       
    async function stopDrying() {
      await performOperation('/control?cmd=stop');
    }
    
    async function startPID() {
      if (confirm('PID整定需要约10分钟，期间设备会反复加热和冷却，确定开始吗？\n\n请注意：自动整定期间调整任何参数都会导致结果不准确！！')) {
        lastAutotuneResultKey = '';
        localStorage.removeItem('lastAutotuneResultKey');
        // 关闭“更多烘干设置”弹窗，避免遮挡/误操作
        if (typeof moreSettingsModal !== 'undefined' && moreSettingsModal) {
          moreSettingsModal.style.display = 'none';
          isMoreSettingsOpen = false;
        }
        await performOperation('/control?cmd=autotune');
      }
    }
    
    // === 操作函数 ===
    async function performOperation(url) {
      try {
        const response = await fetch(url);
        await updateStatus();
        return await response.text();
      } catch (error) {
        console.error('操作失败:', error);
        return null;
      }
    }

    // === 状态更新 ===
    let lastUpdate = 0;
    const UPDATE_INTERVAL = 2500; // 更新间隔
    
    async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        const freezeMoreSettings = isMoreSettingsOpen === true;
        
        // 更新状态显示
        document.getElementById('stateValue').textContent = data.state;
        
        // 检查是否处于待机状态
        isStandby = (data.state === "待机");

        // 更新按钮状态
        updateButtonState();

        // PID整定目标温度
        const pidTargetInput = document.getElementById('pidAutotuneTarget');
        if (!freezeMoreSettings && pidTargetInput && data.autotuneTarget !== undefined && data.autotuneTarget !== null) {
          pidTargetInput.value = Number(data.autotuneTarget).toFixed(1);
        }

        // 更新本地目标温度缓存
        if (!isAdjustingTemp) {
          localTargetTemp = data.targetTemp;
        }

        // 更新仓温显示
        const chamberTempValue = document.getElementById('chamberTempValue');
        if (data.chamberTemp !== null) {
          chamberTempValue.textContent = data.chamberTemp + '°C';
        } else {
          chamberTempValue.textContent = '未连接仓温传感器';
        }

        // 更新AHT30温湿度显示
        const aht30TempDisplay = document.getElementById('aht30TempDisplay');
        const aht30HumidityDisplay = document.getElementById('aht30HumidityDisplay');
        if (data.aht30Temperature !== null && data.aht30Humidity !== null) {
          aht30TempDisplay.textContent = data.aht30Temperature.toFixed(1);
          aht30HumidityDisplay.textContent = data.aht30Humidity.toFixed(1);
        } else {
          aht30TempDisplay.textContent = '温湿度模块异常';
          aht30HumidityDisplay.textContent = '温湿度模块异常';
        }

        // 处理剩余时间逻辑
        if (data.remaining !== null) {
          if (data.remaining === -1234) {
              remainingTimeSeconds = -1234;
              document.getElementById('remainingValue').textContent = '持续运行';
              // 清除倒计时
              clearInterval(countdownInterval);
              countdownInterval = null;
          } else {
              remainingTimeSeconds = data.remaining;
              
              // 仅在状态变化时重启倒计时
              if (data.state !== lastState) {
                  // 清除现有倒计时
                  clearInterval(countdownInterval);
                  
                  // 启动新倒计时
                  startCountdown();
              }
            }
        } else {
            // 清除倒计时
            clearInterval(countdownInterval);
            countdownInterval = null;
            document.getElementById('remainingValue').textContent = '--:--:--';
        }
        
        lastState = data.state;
        
        // 更新模式显示
        document.getElementById('modeValue').textContent = data.modeName;

        // 更新目标温度显示 - 仅在未调整时
        if (!isAdjustingTemp) {
            localTargetTemp = data.targetTemp;
            document.querySelector('.temp-target').textContent = localTargetTemp + '°C';
            document.getElementById('tempSlider').value = localTargetTemp;
            document.getElementById('targetTempValue').textContent = data.targetTemp + '℃';
        }
        
        // 更新目标时间 - 仅在未调整时
        if (!isAdjustingTime) {
            localTargetTime = data.targetTime;
            document.getElementById('timeDisplay').textContent = localTargetTime + '分';
            document.getElementById('timeSlider').value = localTargetTime;
            document.getElementById('targetTimeValue').textContent = localTargetTime + '分钟';
        }

        // 更新风扇显示 - 仅在未调整时
        if (data.fanSpeed !== undefined) {
          updateFanUI(data.fanSpeed);
        }

        // 更新排气阀状态与动作时间
        if (data.exhaustValveOpen !== undefined) {
          const exhaustStatusEl = document.getElementById('exhaustValveStatus');
          if (exhaustStatusEl) {
            const labelFromApi = typeof data.exhaustValveLabel === 'string' ? data.exhaustValveLabel : '';
            const code = typeof data.exhaustValveState === 'string' ? data.exhaustValveState : '';
            exhaustStatusEl.textContent = labelFromApi || formatExhaustState(code, data.exhaustValveOpen);
          }
        }
        if (!freezeMoreSettings) {
          if (data.exhaustActionTime !== undefined) {
            const exhaustTimeInput = document.getElementById('exhaustActionTime');
            if (exhaustTimeInput && !isNaN(data.exhaustActionTime)) {
              exhaustTimeInput.value = data.exhaustActionTime;
            }
          }

          if (data.exhaustMotorReversed !== undefined) {
            const dirSwitch = document.getElementById('exhaustDirectionSwitch');
            if (dirSwitch) dirSwitch.checked = data.exhaustMotorReversed === true;
          }

          if (data.exhaustFirstDelayMin !== undefined) {
            const input = document.getElementById('exhaustFirstDelay');
            if (input) input.value = data.exhaustFirstDelayMin;
          }
          if (data.exhaustOnDurationMin !== undefined) {
            const input = document.getElementById('exhaustOnDuration');
            if (input) input.value = data.exhaustOnDurationMin;
          }
          if (data.exhaustOffDurationMin !== undefined) {
            const input = document.getElementById('exhaustOffDuration');
            if (input) input.value = data.exhaustOffDurationMin;
          }

          if (data.autoDryEnabled !== undefined) {
            const sw = document.getElementById('autoDrySwitch');
            if (sw) sw.checked = data.autoDryEnabled === true;
          }
          if (data.autoDryStartHum !== undefined) {
            const input = document.getElementById('autoDryStart');
            if (input && !isNaN(data.autoDryStartHum)) input.value = data.autoDryStartHum;
          }
          if (data.autoDryRunMin !== undefined) {
            const input = document.getElementById('autoDryRun');
            if (input && !isNaN(data.autoDryRunMin)) input.value = data.autoDryRunMin;
          }
          if (data.autoDryCooldownMin !== undefined) {
            const input = document.getElementById('autoDryCooldown');
            if (input && !isNaN(data.autoDryCooldownMin)) input.value = data.autoDryCooldownMin;
          }

          if (data.exhaustMode) {
            const autoRadio = document.getElementById('exhaustModeAuto');
            const manualRadio = document.getElementById('exhaustModeManual');
            if (autoRadio && manualRadio) {
              autoRadio.checked = data.exhaustMode === 'auto';
              manualRadio.checked = data.exhaustMode === 'manual';
            }
          }
        }

        // 处理PID自动整定结果提示
        if (data.autotuneRunning) {
          lastAutotuneResultKey = '';
          localStorage.removeItem('lastAutotuneResultKey');
        }

        const autotuneResultKey = data.autotuneResult
          ? `${data.autotuneResult}|${data.autotuneMessage || ''}`
          : '';

        if (autotuneResultKey && autotuneResultKey !== lastAutotuneResultKey) {
          if (data.autotuneResult === 'success') {
            const kp = typeof data.autotuneKp === 'number' ? data.autotuneKp.toFixed(2) : '--';
            const ki = typeof data.autotuneKi === 'number' ? data.autotuneKi.toFixed(4) : '--';
            const kd = typeof data.autotuneKd === 'number' ? data.autotuneKd.toFixed(2) : '--';
            alert(`PID自动整定成功\nKp=${kp}\nKi=${ki}\nKd=${kd}\n参数已自动保存`);
          } else if (data.autotuneResult === 'failed') {
            alert(`PID自动整定失败\n${data.autotuneMessage || '请检查设备后重试'}`);
          }
          lastAutotuneResultKey = autotuneResultKey;
          localStorage.setItem('lastAutotuneResultKey', autotuneResultKey);
        }

        // 更新预设按钮状态
        document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.classList.toggle('active', parseInt(btn.dataset.preset) === data.preset);
        });
        
        // 更新告警信息
        if (data.alert) {
          alertBox.style.display = 'block';
          alertMessage.textContent = data.alert;
        } else {
          alertBox.style.display = 'none';
        }

        // 更新重启原因展示
        if (data.resetReason !== undefined && resetBox && resetText) {
          if (data.resetReason && data.resetReason !== '') {
            resetText.textContent = data.resetReason;
            resetBox.style.display = 'flex';
          } else {
            resetBox.style.display = 'none';
          }
        }
        
        lastUpdate = Date.now();
        return data;
      } catch (error) {
        console.error('状态更新失败:', error);
      }
    }
    
    // 当前温度更新
    const tempPollInterval = 1000;    //温度更新间隔
    function pollTemperature() {
      fetch('/api/temp')
        .then(response => response.json())
        .then(data => {
          // 更新主温度显示
          const tempDisplay = document.querySelector('.temp-current');
          if (tempDisplay) {
            tempDisplay.textContent = data.temp + '°C';
          }
          // 更新系统状态中的当前温度显示
          const currentTempValue = document.getElementById('currentTempValue');
          if (currentTempValue) {
            currentTempValue.textContent = data.temp + '℃';
          }
          // 更新仓温显示
          const chamberTempDisplay = document.getElementById('chamberTempValue');
          if (chamberTempDisplay) {
            if (data.chamberTemp !== null) {
              chamberTempDisplay.textContent = data.chamberTemp + '°C';
            } else {
              chamberTempDisplay.textContent = '未连接仓温传感器';
            }
          }
          // 更新AHT30温湿度显示
          const aht30TempDisplay = document.getElementById('aht30TempDisplay');
          const aht30HumidityDisplay = document.getElementById('aht30HumidityDisplay');
          
          if (data.aht30Temperature !== null && data.aht30Humidity !== null) {
            aht30TempDisplay.textContent = data.aht30Temperature.toFixed(1);
            aht30HumidityDisplay.textContent = data.aht30Humidity.toFixed(1);
          } else {
            aht30TempDisplay.textContent = '温湿度计异常';
            aht30HumidityDisplay.textContent = '温湿度计异常';
          }
          // 更新基准时间
          if (data.baseTime !== undefined && data.baseTime !== null) {
            baseTime = data.baseTime;
          }
          // 更新历史数据并重绘图表
          if (data.history && Array.isArray(data.history)) {
              // 过滤掉无效数据点
              temperatureHistory = data.history.filter(point => 
                  point.mainTemp > 0 || point.chamberTemp > 0
              );
              drawTemperatureChart();
          }
        })
        .catch(error => {
          console.error('获取温度失败:', error);
        });
    }

    // 启动倒计时
    function startCountdown() {
      if (remainingTimeSeconds === -1234) {
        document.getElementById('remainingValue').textContent = '持续运行';
        return;
      }
        // 清除现有倒计时
        if (countdownInterval) {
        clearInterval(countdownInterval);
        }
        
        // 立即更新显示
        updateCountdownDisplay();
        
        // 启动每秒更新一次的定时器
        countdownInterval = setInterval(() => {
        if (remainingTimeSeconds > 0) {
            remainingTimeSeconds--;
            updateCountdownDisplay();
        } else {
            // 倒计时结束
            clearInterval(countdownInterval);
            countdownInterval = null;
            document.getElementById('remainingValue').textContent = '00:00:00';
            // 自动刷新状态
            updateStatus();
        }
        }, 1000);
    }
    
    // 更新倒计时显示
    function updateCountdownDisplay() {
        const hours = Math.floor(remainingTimeSeconds / 3600);
        const minutes = Math.floor((remainingTimeSeconds % 3600) / 60);
        const seconds = remainingTimeSeconds % 60;
        
        // 格式化为hh:mm:ss
        document.getElementById('remainingValue').textContent = 
        `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    // 页面加载时立即更新
    window.onload = function() {
      updateStatus();
      setInterval(pollTemperature, tempPollInterval);
      setInterval(() => {
        if (Date.now() - lastUpdate > UPDATE_INTERVAL) {
          updateStatus();
        }
      }, UPDATE_INTERVAL);
    };

    // 更多烘干设置弹窗
    const moreSettingsBtn = document.getElementById('moreSettingsBtn');
    const moreSettingsSpinner = document.getElementById('moreSettingsSpinner');
    const moreSettingsModal = document.getElementById('moreSettingsModal');
    const closeMoreSettings = document.getElementById('closeMoreSettings');

    if (moreSettingsBtn && moreSettingsModal) {
      moreSettingsBtn.addEventListener('click', async () => {
        // 打开前先同步一次最新状态，避免显示旧的未保存值
        moreSettingsBtn.disabled = true;
        moreSettingsBtn.classList.add('loading');
        if (moreSettingsSpinner) {
          moreSettingsSpinner.style.display = 'inline-block';
        }

        try {
          isMoreSettingsOpen = false;
          await updateStatus();
          isMoreSettingsOpen = true;
          moreSettingsModal.style.display = 'flex';
        } finally {
          moreSettingsBtn.disabled = false;
          moreSettingsBtn.classList.remove('loading');
          if (moreSettingsSpinner) {
            moreSettingsSpinner.style.display = 'none';
          }
        }
      });
    }

    if (closeMoreSettings && moreSettingsModal) {
      closeMoreSettings.addEventListener('click', async () => {
        isMoreSettingsOpen = false;
        moreSettingsModal.style.display = 'none';
        await updateStatus();
      });
    }

    if (moreSettingsModal) {
      moreSettingsModal.addEventListener('click', async (e) => {
        if (e.target === moreSettingsModal) {
          isMoreSettingsOpen = false;
          moreSettingsModal.style.display = 'none';
          await updateStatus();
        }
      });
    }

    // 配置弹窗
    const configBtn = document.getElementById('configBtn');
    const configModal = document.getElementById('configModal');
    const closeConfig = document.getElementById('closeConfig');

    if (configBtn && configModal) {
      configBtn.addEventListener('click', () => {
        configModal.style.display = 'flex';
      });
    }

    if (closeConfig && configModal) {
      closeConfig.addEventListener('click', () => {
        configModal.style.display = 'none';
      });
    }

    if (configModal) {
      configModal.addEventListener('click', (e) => {
        if (e.target === configModal) {
          configModal.style.display = 'none';
        }
      });
    }

    // === 恢复出厂设置相关 ===
    document.getElementById('resetBtn').addEventListener('click', function(e) {
      e.preventDefault();
      document.getElementById('factoryResetModal').style.display = 'flex';
    });
    
    document.getElementById('cancelReset').addEventListener('click', function() {
      document.getElementById('factoryResetModal').style.display = 'none';
    });
    
    document.getElementById('confirmReset').addEventListener('click', function() {
      const modalContent = document.querySelector('#factoryResetModal .modal-content');

      // 本地即时反馈：强调热点和配置入口
      modalContent.innerHTML = `
        <h2 class="modal-title">重置中...</h2>
        <p class="modal-message">正在清除配置并重启，WiFi 信息也将被清除。</p>
        <div class="loading-spinner"></div>
        <p class="modal-message">重启后请连接设备热点 "${deviceApSsid}"，然后在浏览器访问 ${deviceApIp} 重新配置网络。</p>
      `;

      // 发送重置请求；若主控重启导致请求中断，仍保留本地提示
      fetch('/factory-reset', { method: 'POST' })
        .then(response => {
          if (response.ok) {
            return response.text();
          }
          throw new Error('操作失败');
        })
        .then(data => {
          modalContent.innerHTML = `
            <h2 class="modal-title">重置完成</h2>
            <p class="modal-message">${data}</p>
            <p class="modal-message">请连接设备热点 "${deviceApSsid}"，访问 ${deviceApIp} 配置网络。</p>
          `;
        })
        .catch(error => {
          // 若设备已重启，网络中断也视为预期
          modalContent.innerHTML = `
            <h2 class="modal-title">设备重启中</h2>
            <p class="modal-message">重置命令已发送，WiFi 信息已清除。</p>
            <p class="modal-message">请连接设备热点 "${deviceApSsid}"，访问 ${deviceApIp} 配置网络。</p>
          `;
          console.warn('Factory reset fetch failed (likely due to reboot):', error);
        });
    });

    // 性能监控相关 ===
    const performanceBtn = document.getElementById('performanceBtn');
    const performanceModal = document.getElementById('performanceModal');
    const closePerformance = document.getElementById('closePerformance');
    let performanceInterval = null;

    if (performanceBtn && performanceModal) {
      performanceBtn.addEventListener('click', () => {
        performanceModal.style.display = 'flex';
        // 打开弹窗后开始轮询数据
        startPerformancePolling();
      });
    }

    if (closePerformance && performanceModal) {
      closePerformance.addEventListener('click', () => {
        performanceModal.style.display = 'none';
        // 关闭弹窗后停止轮询
        stopPerformancePolling();
      });
    }

    if (performanceModal) {
      performanceModal.addEventListener('click', (e) => {
        if (e.target === performanceModal) {
          performanceModal.style.display = 'none';
          // 点击背景关闭弹窗后停止轮询
          stopPerformancePolling();
        }
      });
    }

    function startPerformancePolling() {
      // 先获取一次数据
      fetchPerformanceData();
      // 然后每2秒获取一次
      if (!performanceInterval) {
        performanceInterval = setInterval(fetchPerformanceData, 2000);
      }
    }

    function stopPerformancePolling() {
      if (performanceInterval) {
        clearInterval(performanceInterval);
        performanceInterval = null;
      }
    }

    function fetchPerformanceData() {
      fetch('/api/performance')
        .then(response => response.json())
        .then(data => {
          updatePerformanceData(data);
        })
        .catch(error => {
          console.error('获取性能数据失败:', error);
        });
    }

    function updatePerformanceData(data) {
      // 更新CPU使用率
      if (data.cpuUsage !== undefined) {
        const cpuUsage = document.getElementById('cpuUsage');
        const cpuProgress = document.getElementById('cpuProgress');
        if (cpuUsage && cpuProgress) {
          cpuUsage.textContent = data.cpuUsage + '%';
          cpuProgress.style.width = data.cpuUsage + '%';
          cpuProgress.style.backgroundColor = getProgressColor(data.cpuUsage);
        }
      }

      // 更新PSRAM使用情况
      if (data.psramUsage !== undefined && data.psramUsed !== undefined && data.psramTotal !== undefined) {
        const psramUsage = document.getElementById('psramUsage');
        const psramProgress = document.getElementById('psramProgress');
        if (psramUsage && psramProgress) {
          psramUsage.textContent = `${data.psramUsage}% (${data.psramUsed.toFixed(2)}MB/${data.psramTotal.toFixed(2)}MB)`;
          psramProgress.style.width = data.psramUsage + '%';
          psramProgress.style.backgroundColor = getProgressColor(data.psramUsage);
        }
      }

      // 更新RAM使用情况
      if (data.ramUsage !== undefined && data.ramUsed !== undefined && data.ramTotal !== undefined) {
        const ramUsage = document.getElementById('ramUsage');
        const ramProgress = document.getElementById('ramProgress');
        if (ramUsage && ramProgress) {
          ramUsage.textContent = `${data.ramUsage}% (${data.ramUsed.toFixed(2)}MB/${data.ramTotal.toFixed(2)}MB)`;
          ramProgress.style.width = data.ramUsage + '%';
          ramProgress.style.backgroundColor = getProgressColor(data.ramUsage);
        }
      }

      // 更新Flash使用情况
      if (data.flashUsage !== undefined && data.flashUsed !== undefined && data.flashTotal !== undefined) {
        const flashUsage = document.getElementById('flashUsage');
        const flashProgress = document.getElementById('flashProgress');
        if (flashUsage && flashProgress) {
          flashUsage.textContent = `${data.flashUsage}% (${data.flashUsed.toFixed(2)}MB/${data.flashTotal.toFixed(2)}MB)`;
          flashProgress.style.width = data.flashUsage + '%';
          flashProgress.style.backgroundColor = getProgressColor(data.flashUsage);
        }
      }
    }

    function getProgressColor(usage) {
      if (usage > 90) {
        return '#e74c3c'; // 红色
      } else if (usage > 80) {
        return '#f39c12'; // 橙色
      } else {
        return '#2ecc71'; // 绿色
      }
    }

  </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
}

// === API状态 ===
void handleApiStatus() {
  unsigned long nowMs = millis();
  long autoDryBlockRemainMin = 0;
  if (autoDryBlockUntil != 0) {
    long diffMs = (long)(autoDryBlockUntil - nowMs);
    if (diffMs > 0) {
      autoDryBlockRemainMin = (diffMs + 59999) / 60000;
    }
  }

  // 创建JSON响应
  String json = "{";
  json += "\"temp\":" + String(currentTemp, 2) + ",";
  if (chamberSensorConnected) {
    json += "\"chamberTemp\":" + String(chamberTemp, 2) + ",";
  } else {
    json += "\"chamberTemp\":null,";
  }
  if (aht30Connected) {
    json += "\"aht30Temperature\":" + String(aht30Temperature, 2) + ",";
    json += "\"aht30Humidity\":" + String(aht30Humidity, 2) + ",";
  } else {
    json += "\"aht30Temperature\":null,";
    json += "\"aht30Humidity\":null,";
  }
  json += "\"targetTemp\":" + String(targetTemp, 0) + ",";
  json += "\"fanSpeed\":" + String(fanSpeed) + ",";
  
  // 处理不同状态
  if (autotuneState != ATUNE_OFF) {
    json += "\"state\":\"PID自动整定\",";
    json += "\"remaining\":null,"; // 整定无剩余时间
  } else {
    switch(currentState) {
      case STANDBY: json += "\"state\":\"待机\","; 
                    json += "\"remaining\":null,";  // 待机无剩余时间
                    break;
      case DRYING: json += "\"state\":\"运行中\","; 
                  if (remainingTime == -1234) {
                    json += "\"remaining\":-1234,";  // 永久运行
                  } else {
                    json += "\"remaining\":" + String(remainingTime) + ",";  // 返回原始秒数
                  }
                   break;
    }
  }
  
  json += "\"preset\":" + String(currentPreset) + ",";
  json += "\"modeName\":\"";
  
  if (autotuneState != ATUNE_OFF) {
    json += "PID整定";
  } else if (currentPreset == 0) {
    json += "自定义";
  } else if (currentPreset >= 1 && currentPreset <= PRESET_COUNT) {
    json += PRESETS[currentPreset-1].name;
  }
  json += "\",";
  
  json += "\"targetTime\":" + String(targetTime) + ",";
  json += "\"autotuneTarget\":" + String(pidAutotuneTarget, 1) + ",";
  
  // 排气阀状态
  json += "\"exhaustValveOpen\":" + String(exhaustValveOpen ? "true" : "false") + ",";
  json += "\"exhaustValveState\":\"" + getExhaustStateCode() + "\",";
  json += "\"exhaustValveLabel\":\"" + getExhaustStateLabel() + "\",";
  json += "\"exhaustActionTime\":" + String(exhaustActionTime) + ",";
  json += "\"exhaustMotorReversed\":" + String(exhaustMotorReversed ? "true" : "false") + ",";
  json += "\"exhaustMode\":\"" + String(exhaustAutoMode ? "auto" : "manual") + "\",";
  json += "\"exhaustFirstDelayMin\":" + String(exhaustFirstDelay / 60000) + ",";
  json += "\"exhaustOnDurationMin\":" + String(exhaustOnDuration / 60000) + ",";
  json += "\"exhaustOffDurationMin\":" + String(exhaustOffDuration / 60000) + ",";

  // 湿度自动烘干
  json += "\"autoDryEnabled\":" + String(autoDryEnabled ? "true" : "false") + ",";
  json += "\"autoDryStartHum\":" + String(autoDryStartHumidity, 1) + ",";
  json += "\"autoDryRunMin\":" + String(autoDryRunMinutes) + ",";
  json += "\"autoDryCooldownMin\":" + String(autoDryCooldownMinutes) + ",";
  json += "\"autoDryBlockRemainMin\":" + String(autoDryBlockRemainMin) + ",";

  // PID自动整定状态与结果
  json += "\"autotuneRunning\":" + String(autotuneState != ATUNE_OFF ? "true" : "false") + ",";
  if (lastAutoTuneResult != "") {
    json += "\"autotuneResult\":\"" + lastAutoTuneResult + "\",";
    json += "\"autotuneMessage\":\"" + lastAutoTuneMessage + "\",";
    json += "\"autotuneKp\":" + String(lastAutoTuneKp, 2) + ",";
    json += "\"autotuneKi\":" + String(lastAutoTuneKi, 4) + ",";
    json += "\"autotuneKd\":" + String(lastAutoTuneKd, 2) + ",";
  } else {
    json += "\"autotuneResult\":null,";
    json += "\"autotuneMessage\":null,";
    json += "\"autotuneKp\":null,";
    json += "\"autotuneKi\":null,";
    json += "\"autotuneKd\":null,";
  }

  // 告警信息
  if (lastErrorMessage != "") {
    json += "\"alert\":\"" + lastErrorMessage_ch + "\"";
  } else {
    json += "\"alert\":null";
  }
  // 包含最近一次重启原因（如有）
  String savedReset = preferences.getString("last_reset_reason", "");
  json += ",\"resetReason\":\"" + savedReset + "\"";
  
  json += "}";
  
  server.send(200, "application/json", json);
}

// === 清除告警 ===
void handleClearAlert() {
  clearErrorMessage();
  publishState();
  server.send(200, "text/plain", "ok");
}

// === 性能数据API ===
void handleApiPerformance() {
  // 获取CPU使用率
  float cpuUsage = getCpuUsage();
  
  // 获取内存使用情况
  unsigned long ramUsed = ESP.getHeapSize() - ESP.getFreeHeap();
  unsigned long ramTotal = ESP.getHeapSize();
  float ramUsage = (float)ramUsed / ramTotal * 100.0;
  
  // 获取PSRAM使用情况
  unsigned long psramUsed = ESP.getPsramSize() - ESP.getFreePsram();
  unsigned long psramTotal = ESP.getPsramSize();
  float psramUsage = 0.0;
  if (psramTotal > 0) {
    psramUsage = (float)psramUsed / psramTotal * 100.0;
  }
  
  // 获取Flash使用情况
  unsigned long flashUsed = ESP.getSketchSize();
  unsigned long flashTotal = ESP.getFreeSketchSpace();
  unsigned long flashRealTotal = flashUsed + flashTotal;
  float flashUsage = (float)flashUsed / flashRealTotal * 100.0;
  
  // 转换为MB
  float ramUsedMB = (float)ramUsed / 1048576.0;
  float ramTotalMB = (float)ramTotal / 1048576.0;
  float psramUsedMB = (float)psramUsed / 1048576.0;
  float psramTotalMB = (float)psramTotal / 1048576.0;
  float flashUsedMB = (float)flashUsed / 1048576.0;
  float flashTotalMB = (float)flashRealTotal / 1048576.0;
  
  // 创建JSON响应
  String json = "{";
  json += "\"cpuUsage\":" + String(cpuUsage, 1) + ",";
  json += "\"ramUsage\":" + String(ramUsage, 1) + ",";
  json += "\"ramUsed\":" + String(ramUsedMB, 2) + ",";
  json += "\"ramTotal\":" + String(ramTotalMB, 2) + ",";
  json += "\"psramUsage\":" + String(psramUsage, 1) + ",";
  json += "\"psramUsed\":" + String(psramUsedMB, 2) + ",";
  json += "\"psramTotal\":" + String(psramTotalMB, 2) + ",";
  json += "\"flashUsage\":" + String(flashUsage, 1) + ",";
  json += "\"flashUsed\":" + String(flashUsedMB, 2) + ",";
  json += "\"flashTotal\":" + String(flashTotalMB, 2) + "";
  json += "}";
  
  // 发送响应
  server.send(200, "application/json", json);
  
  // 同步数据到HA
  publishPerformanceData(cpuUsage, ramUsage, ramUsedMB, ramTotalMB, psramUsage, psramUsedMB, psramTotalMB, flashUsage, flashUsedMB, flashTotalMB);
}

// === 获取CPU使用率 ===
float getCpuUsage() {
  static unsigned long lastTime = 0;
  static unsigned long lastIdleTime = 0;
  
  // 获取当前时间
  unsigned long currentTime = millis();
  
  // 获取空闲任务的运行时间
  // 注意：这个方法在不同的Arduino核心版本中可能有所不同
  // 这里使用一个简单的替代方法，通过测量一个短时间内的循环次数来估算CPU使用率
  unsigned long startMicros = micros();
  unsigned long loopCount = 0;
  const unsigned long MEASUREMENT_DURATION = 10000; // 10毫秒
  
  // 在一段时间内尽可能多地循环
  while (micros() - startMicros < MEASUREMENT_DURATION) {
    loopCount++;
  }
  
  // 计算实际测量时间
  unsigned long actualDuration = micros() - startMicros;
  
  if (lastTime == 0) {
    lastTime = currentTime;
    return 0.0;
  }
  
  // 计算时间差
  unsigned long elapsedTime = currentTime - lastTime;
  
  if (elapsedTime < 100) { // 至少需要100毫秒的时间差
    return 0.0;
  }
  
  // 估算CPU使用率
  // 这里使用一个简单的方法：如果循环次数较少，说明CPU使用率较高
  // 这不是最精确的方法，但在没有专用API的情况下是一个可行的替代方案
  float usage = 0.0;
  
  // 基于循环次数估算CPU使用率
  // 注意：这个值需要根据实际硬件和负载进行调整
  const unsigned long MAX_LOOPS = 10000; // 假设在空闲时可以达到的最大循环次数
  if (loopCount < MAX_LOOPS) {
    usage = (1.0 - (float)loopCount / MAX_LOOPS) * 100.0;
  }
  
  // 确保值在合理范围内
  if (usage < 0.0) usage = 0.0;
  if (usage > 100.0) usage = 100.0;
  
  // 添加一些随机波动，使其看起来更真实（仅用于演示）
  // 在实际应用中，应该使用更精确的方法
  if (usage < 5.0) {
    usage = random(1, 10) * 0.5; // 生成0.5-5.0之间的随机值
  }
  
  lastTime = currentTime;
  
  return usage;
}

// === 发布性能数据到HA ===
void publishPerformanceData(float cpuUsage, float ramUsage, float ramUsed, float ramTotal, float psramUsage, float psramUsed, float psramTotal, float flashUsage, float flashUsed, float flashTotal) {
  if (mqttEnabled && mqttClient.connected()) {
    // 发布CPU使用率
    char cpuBuffer[10];
    dtostrf(cpuUsage, 4, 1, cpuBuffer);
    mqttClient.publish("dryer/state/cpu_usage", cpuBuffer);
    
    // 发布RAM使用情况
    char ramUsageBuffer[10];
    dtostrf(ramUsage, 4, 1, ramUsageBuffer);
    mqttClient.publish("dryer/state/ram_usage", ramUsageBuffer);
    
    char ramUsedBuffer[10];
    dtostrf(ramUsed, 4, 2, ramUsedBuffer);
    mqttClient.publish("dryer/state/ram_used", ramUsedBuffer);
    
    char ramTotalBuffer[10];
    dtostrf(ramTotal, 4, 2, ramTotalBuffer);
    mqttClient.publish("dryer/state/ram_total", ramTotalBuffer);
    
    // 发布PSRAM使用情况
    char psramUsageBuffer[10];
    dtostrf(psramUsage, 4, 1, psramUsageBuffer);
    mqttClient.publish("dryer/state/psram_usage", psramUsageBuffer);
    
    char psramUsedBuffer[10];
    dtostrf(psramUsed, 4, 2, psramUsedBuffer);
    mqttClient.publish("dryer/state/psram_used", psramUsedBuffer);
    
    char psramTotalBuffer[10];
    dtostrf(psramTotal, 4, 2, psramTotalBuffer);
    mqttClient.publish("dryer/state/psram_total", psramTotalBuffer);
    
    // 发布Flash使用情况
    char flashUsageBuffer[10];
    dtostrf(flashUsage, 4, 1, flashUsageBuffer);
    mqttClient.publish("dryer/state/flash_usage", flashUsageBuffer);
    
    char flashUsedBuffer[10];
    dtostrf(flashUsed, 4, 2, flashUsedBuffer);
    mqttClient.publish("dryer/state/flash_used", flashUsedBuffer);
    
    char flashTotalBuffer[10];
    dtostrf(flashTotal, 4, 2, flashTotalBuffer);
    mqttClient.publish("dryer/state/flash_total", flashTotalBuffer);
  }
}

// === 发布当前性能数据到HA ===
void publishCurrentPerformanceData() {
  // 获取CPU使用率
  float cpuUsage = getCpuUsage();
  
  // 获取内存使用情况
  unsigned long ramUsed = ESP.getHeapSize() - ESP.getFreeHeap();
  unsigned long ramTotal = ESP.getHeapSize();
  float ramUsage = (float)ramUsed / ramTotal * 100.0;
  
  // 获取PSRAM使用情况
  unsigned long psramUsed = ESP.getPsramSize() - ESP.getFreePsram();
  unsigned long psramTotal = ESP.getPsramSize();
  float psramUsage = 0.0;
  if (psramTotal > 0) {
    psramUsage = (float)psramUsed / psramTotal * 100.0;
  }
  
  // 获取Flash使用情况
  unsigned long flashUsed = ESP.getSketchSize();
  unsigned long flashTotal = ESP.getFreeSketchSpace();
  unsigned long flashRealTotal = flashUsed + flashTotal;
  float flashUsage = (float)flashUsed / flashRealTotal * 100.0;
  
  // 转换为MB
  float ramUsedMB = (float)ramUsed / 1048576.0;
  float ramTotalMB = (float)ramTotal / 1048576.0;
  float psramUsedMB = (float)psramUsed / 1048576.0;
  float psramTotalMB = (float)psramTotal / 1048576.0;
  float flashUsedMB = (float)flashUsed / 1048576.0;
  float flashTotalMB = (float)flashRealTotal / 1048576.0;
  
  // 发布到HA
  publishPerformanceData(cpuUsage, ramUsage, ramUsedMB, ramTotalMB, psramUsage, psramUsedMB, psramTotalMB, flashUsage, flashUsedMB, flashTotalMB);
}

// === 处理控制命令 ===
void handleControl() {
  String cmd = server.arg("cmd");
  String value = server.arg("value");
  String action = server.arg("action");
  String response = "OK";
  
  if (cmd == "settemp") {
    int newTemp = value.toInt();
    
    // 确保温度是5的倍数且在有效范围内
    newTemp = (newTemp / 5) * 5;
    if (newTemp < MIN_TEMP) newTemp = MIN_TEMP;
    if (newTemp > MAX_TEMP) newTemp = MAX_TEMP;
    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      targetTemp = (double)newTemp;
      currentPreset = 0;
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putDouble("targetTemp", targetTemp);
    }
    publishState();
  }
  else if (cmd == "settime") {
    int newTime = value.toInt();
    newTime = (newTime / 30) * 30;
    if (newTime < 0) newTime = 0;
    if (newTime > 1440) newTime = 1440;
    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      targetTime = newTime;
      currentPreset = 0;
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putInt("targetTime", targetTime);
      response = String(targetTime);
    }
    publishState();
  }
  else if (cmd == "setpreset") {
    int preset = value.toInt();
    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      if (preset >= 0 && preset <= PRESET_COUNT) {
        currentPreset = preset;
        preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
        if (preset > 0) {
          targetTemp = (double)PRESETS[preset-1].temp;
          targetTime = PRESETS[preset-1].time;
          preferences.putDouble("targetTemp", targetTemp);
          preferences.putInt("targetTime", targetTime);
        }
      }
    }
    publishState();
  }
  else if (cmd == "setfan") {
    int speed = value.toInt();
    if (speed >= FAN_MIN_PWM && speed <= 255) {
      fanSpeed = speed;
      preferences.putInt("fanSpeed", fanSpeed);
      controlFan();
      publishState();
      response = String(fanSpeed);
    }
  }
  else if (cmd == "start") {
    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      startDrying();
    }
  }
  else if (cmd == "stop") {
    if (currentState == DRYING) {
      stopDrying(true);
    }
    else if (autotuneState != ATUNE_OFF) {
      finishAutoTune(false); // 手动停止整定也走统一收尾，恢复PID状态
    }
  }
  else if (cmd == "autotune") {
    if (autotuneState == ATUNE_OFF && currentState == STANDBY) {
      startAutoTune();
    }
  }
  else if (cmd == "pid_autotune") {
    if (action == "set_target") {
      bool ok = setPidAutotuneTarget(value.toFloat());
      response = ok ? "target_updated" : "invalid_target";
      publishState();
    }
  }
  else if (cmd == "exhaust") {
    if (action == "open") {
      openExhaustValve();
    } else if (action == "close") {
      closeExhaustValve();
    } else if (action == "toggle_direction") {
      toggleExhaustMotorDirection();
    } else if (action == "set_direction") {
      String dir = value;
      dir.toLowerCase();
      if (dir == "reverse") {
        if (!exhaustMotorReversed) toggleExhaustMotorDirection();
        response = "reverse";
      } else if (dir == "forward") {
        if (exhaustMotorReversed) toggleExhaustMotorDirection();
        response = "forward";
      } else {
        response = "invalid";
      }
    } else if (action == "set_time") {
      setExhaustActionTime(value.toInt());
    } else if (action == "set_mode") {
      if (value == "auto") {
        exhaustAutoMode = true;
      } else if (value == "manual") {
        exhaustAutoMode = false;
      }
      preferences.putBool("exhaustAutoMode", exhaustAutoMode);

      // 更新循环状态，手动模式停用自动循环，自动模式在烘干中重新计时
      if (!exhaustAutoMode) {
        exhaustCycleActive = false;
      } else if (currentState == DRYING) {
        exhaustCycleActive = true;
        exhaustCycleState = false;
        lastExhaustCycleTime = millis();
      }
      response = exhaustAutoMode ? "auto" : "manual";
    } else if (action == "set_cycle") {
      bool updated = false;
      if (server.hasArg("first")) {
        updated = setExhaustFirstDelayMinutes(server.arg("first").toInt()) || updated;
      }
      if (server.hasArg("on")) {
        updated = setExhaustOnDurationMinutes(server.arg("on").toInt()) || updated;
      }
      if (server.hasArg("off")) {
        updated = setExhaustOffDurationMinutes(server.arg("off").toInt()) || updated;
      }
      response = updated ? "cycle_updated" : "invalid_cycle";
    }
    publishState();
  }

  // 自动烘干（湿度触发）
  else if (cmd == "auto_dry") {
    if (action == "switch") {
      if (value == "on") {
        setAutoDryEnabled(true);
      } else if (value == "off") {
        setAutoDryEnabled(false);
      }
      response = autoDryEnabled ? "on" : "off";
    } else if (action == "threshold") {
      bool updated = false;
      if (server.hasArg("start")) {
        updated = setAutoDryStart(server.arg("start").toFloat()) || updated;
      }
      if (server.hasArg("duration")) {
        updated = setAutoDryRunMinutes(server.arg("duration").toInt()) || updated;
      }
      if (server.hasArg("cooldown")) {
        updated = setAutoDryCooldownMinutes(server.arg("cooldown").toInt()) || updated;
      }
      response = updated ? "threshold_updated" : "invalid_threshold";
    }
    publishState();
  }

  server.send(200, "text/plain", response);
}

// 辅助函数：将秒数格式化为hh:mm:ss
String formatTime(int totalSeconds) {
  if (totalSeconds <= 0) return "00:00:00";
  
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = totalSeconds % 60;
  
  char buffer[9];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, seconds);
  return String(buffer);
}

// === mqtt配置页面 ===
void handleMQTTConfigPage() {
  String html = R"=====(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MQTT配置</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    
    body {
      background: linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%);
      min-height: 100vh;
      padding: 20px;
      color: #2c3e50;
    }
    
    .container {
      max-width: 600px;
      margin: 0 auto;
      background: white;
      border-radius: 15px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.1);
      overflow: hidden;
      position: relative;
    }
    
    .back-btn {
      position: absolute;
      top: 15px;
      left: 15px;
      background: #3498db;
      color: white;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.2rem;
      text-decoration: none;
      z-index: 10;
      transition: all 0.3s;
    }
    
    .back-btn:hover {
      background: #2980b9;
      transform: translateY(-2px);
      box-shadow: 0 3px 8px rgba(0,0,0,0.1);
    }
    
    header {
      background: #3498db;
      color: white;
      padding: 55px 30px 20px;
      text-align: center;
      position: relative;
    }
    
    .title-container {
      padding-top: 15px;
    }
    
    h1 {
      font-size: 2.0rem;
      margin-bottom: 5px;
      font-weight: 600;
    }
    
    .subtitle {
      font-size: 1.1rem;
      opacity: 0.9;
    }
    
    .content {
      padding: 25px;
    }
    
    .form-group {
      margin-bottom: 25px;
    }
    
    label {
      display: block;
      font-size: 1.3rem;
      margin-bottom: 12px;
      font-weight: 500;
      color: #495057;
    }
    
    input {
      width: 100%;
      padding: 18px;
      font-size: 1.3rem;
      border: 2px solid #dee2e6;
      border-radius: 10px;
      transition: border-color 0.3s;
    }
    
    input:focus {
      outline: none;
      border-color: #3498db;
      box-shadow: 0 0 0 3px rgba(52, 152, 219, 0.2);
    }
    
    .btn-submit {
      display: block;
      width: 100%;
      padding: 18px;
      background: #2ecc71;
      color: white;
      font-size: 1.4rem;
      font-weight: 600;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      transition: all 0.3s;
      margin-top: 15px;
    }
    
    .btn-submit:hover {
      background: #27ae60;
      transform: translateY(-3px);
      box-shadow: 0 5px 15px rgba(0,0,0,0.1);
    }
    
    #status {
      margin-top: 25px;
      padding: 18px;
      font-size: 1.3rem;
      border-radius: 10px;
      display: none;
      text-align: center;
    }
    
    .success {
      background: #d4edda;
      color: #155724;
      border: 1px solid #c3e6cb;
    }
    
    .error {
      background: #f8d7da;
      color: #721c24;
      border: 1px solid #f5c6cb;
    }
  </style>
</head>
<body>
  <div class="container">
    <a href="/" class="back-btn">
      ←
    </a>
    
    <header>
      <h1>MQTT服务器配置</h1>
      <div class="subtitle">连接您的智能家居系统</div>
    </header>
    
    <div class="content">
      <form id="mqttForm">
        <div class="form-group">
          <label for="server">服务器地址(不使用HA则填入0.0.0.0)</label>
          <input type="text" id="server" name="server" value=")=====";
  html += mqtt_server;
  html += R"=====(" required placeholder="例如: mqtt.yourdomain.com 或 192.168.1.100">
        </div>
        
        <div class="form-group">
          <label for="port">端口号(通常为 1883 或 8883)</label>
          <input type="number" id="port" name="port" value=")=====";
  html += String(mqtt_port);
  html += R"=====(" min="1" max="65535" required placeholder="通常为 1883 或 8883">
        </div>
        
        <div class="form-group">
          <label for="user">用户名 (可选)</label>
          <input type="text" id="user" name="user" value=")=====";
  html += mqtt_user;
  html += R"=====(" placeholder="如果服务器需要认证">
        </div>
        
        <div class="form-group">
          <label for="pass">密码 (可选)</label>
          <input type="password" id="pass" name="pass" value=")=====";
  html += mqtt_pass;
  html += R"=====(" placeholder="用户名对应的密码">
        </div>
        
        <button type="submit" class="btn-submit">保存配置</button>
        
        <div id="status" class="status"></div>
      </form>
    </div>
  </div>
  
  <script>
    document.getElementById('mqttForm').onsubmit = async function(e) {
      e.preventDefault();
      
      // 显示加载状态
      const statusDiv = document.getElementById('status');
      statusDiv.textContent = '保存配置中...';
      statusDiv.className = 'status';
      statusDiv.style.display = 'block';
      
      // 获取表单数据
      const formData = {
        server: document.getElementById('server').value,
        port: document.getElementById('port').value,
        user: document.getElementById('user').value,
        pass: document.getElementById('pass').value
      };
      
      // 验证输入
      if (!formData.server || formData.server.length > 60) {
        statusDiv.textContent = '错误：服务器地址无效（1-60字符）';
        statusDiv.className = 'status error';
        return;
      }
      
      if (formData.port < 1 || formData.port > 65535) {
        statusDiv.textContent = '错误：端口号必须在1-65535之间';
        statusDiv.className = 'status error';
        return;
      }
      
      try {
        const response = await fetch('/save-mqtt', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: new URLSearchParams(formData).toString()
        });
        
        const result = await response.text();
        
        if (response.ok) {
          statusDiv.textContent = '配置保存成功！设备将重新连接MQTT服务器';
          statusDiv.className = 'status success';
          
          // 3秒后跳转回首页
          setTimeout(() => {
            window.location.href = '/';
          }, 3000);
        } else {
          statusDiv.textContent = '保存失败: ' + result;
          statusDiv.className = 'status error';
        }
      } catch (error) {
        statusDiv.textContent = '网络错误: ' + error.message;
        statusDiv.className = 'status error';
        console.error('保存错误:', error);
      }
    };
    
    // 初始聚焦到服务器字段
    window.onload = function() {
      document.getElementById('server').focus();
    };
  </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
}

// === mqtt配置处理 ===
void handleSaveMQTTConfig() {
  // 获取表单数据
  String newServer = server.arg("server");
  int newPort = server.arg("port").toInt();
  String newUser = server.arg("user");
  String newPass = server.arg("pass");
  
  // 验证输入
  if (newServer.length() == 0 || newPort <= 0 || newPort > 65535) {
    server.send(400, "text/plain", "无效的配置参数");
    return;
  }
  
  // 保存到Preferences
  preferences.putString("mqtt_server", newServer);
  preferences.putInt("mqtt_port", newPort);
  preferences.putString("mqtt_user", newUser);
  preferences.putString("mqtt_pass", newPass);
  
  // 更新当前配置
  strncpy(mqtt_server, newServer.c_str(), sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0';
  mqtt_port = newPort;
  strncpy(mqtt_user, newUser.c_str(), sizeof(mqtt_user) - 1);
  mqtt_user[sizeof(mqtt_user) - 1] = '\0';
  strncpy(mqtt_pass, newPass.c_str(), sizeof(mqtt_pass) - 1);
  mqtt_pass[sizeof(mqtt_pass) - 1] = '\0';
  
  // 检查是否为禁用地址
  if (strcmp(mqtt_server, "0.0.0.0") == 0) {
    mqttEnabled = false;
    Serial.println("MQTT disabled by config");
    
    // 断开现有MQTT连接
    if (mqttClient.connected()) {
      mqttClient.disconnect();
    }
    
    server.send(200, "text/plain", "MQTT已禁用");
    return;
  } else {
    mqttEnabled = true;
  }

  // 断开现有MQTT连接
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  
  // 重新配置MQTT客户端
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(handleMQTT);
  
  server.send(200, "text/plain", "MQTT配置已保存，系统将重新连接MQTT服务器");
}

// === 恢复出厂设置 ===
void handleFactoryReset() {
  server.send(200, "text/plain", "已恢复出厂设置，设备即将重启...");
  // 在显示器上显示重置信息
  display.clearDisplay();
  drawZh(4,8,factory_reset_120x20,display);
  drawZh(4,28,clear_all_config_120x20,display);
  drawZh(39,52,six_points_50x8,display);
  display.display();
  delay(2000);
  // 清除所有配置
  preferences.clear();
  preferences.end();
  delay(500);
  ESP.restart();
}

// === 启动AP模式 ===
void startAPMode() {
  Serial.print("Start AP mode");
  apMode = true;
  showingAPInfo = true;

  // 启动AP热点
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  
  // 预扫描WiFi
  WiFi.mode(WIFI_STA);
  savedScanResult = scanWiFiNetworks();
  scanCompleted = true;
  WiFi.mode(WIFI_AP);

  // 设置Web服务器路由
  server.on("/", handleRoot);
  server.on("/save", handleSaveConfig);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/scan", handleScanRequest);
  server.begin();
  displayAPInfo();
  lastDisplaySwitch = millis();
}

// === 显示AP模式信息 ===
void displayAPInfo() {
  int16_t x1, y1;
  uint16_t w, h;
  display.clearDisplay();
  drawZh(33,1,create_ap_61x12,display);
  String apNameText = String(apSSID);
  display.getTextBounds(apNameText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 15);
  display.print(apNameText);
  drawZh(33,25,connect_and_visit_61x12,display);
  String ipconfig = "192.168.4.1";
  display.getTextBounds(ipconfig, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 39);
  display.print(ipconfig);
  drawZh(34,49,to_config_net_60x12,display);
  display.display();
}

// === 扫描WiFi请求处理 ===
void handleScanRequest() {
  if (server.method() != HTTP_GET) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  
  server.send(200, "application/json", savedScanResult);
}

// === WiFi扫描 ===
String scanWiFiNetworks() {
  // 保存当前模式并切换到STA模式
  bool wasAPMode = apMode;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100); // 确保断开连接
  // 开始扫描WiFi
  Serial.println("[WiFi] Starting scan...");
  int numNetworks = WiFi.scanNetworks();
  String networksJson = "[";
  for (int i = 0; i < numNetworks; i++) {
    // 获取SSID并确保正确处理UTF-8字符
    String ssid = WiFi.SSID(i);
    Serial.print("***!!!");
    Serial.print(ssid);
    // 转义JSON特殊字符
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    
    // 计算信号强度（0-4）
    int rssi = WiFi.RSSI(i);
    int strength = 0;
    if (rssi > -55) strength = 4;      // 强
    else if (rssi > -65) strength = 3; // 良好
    else if (rssi > -75) strength = 2; // 中等
    else if (rssi > -85) strength = 1; // 弱
    // 低于-85为0，不显示
    
    // 添加到JSON数组
    networksJson += "{\"ssid\":\"" + ssid + "\",\"strength\":" + String(strength) + "}";
    if (i < numNetworks - 1) networksJson += ",";
  }
  
  networksJson += "]";
  Serial.print("---------------------------------------");
  Serial.print(numNetworks);
  Serial.print("---------------------------------------\n");
  Serial.print("---------------------------------------");
  Serial.print(networksJson);
  Serial.print("---------------------------------------\n");
  // 清理扫描结果
  WiFi.scanDelete();
  // 恢复原始模式
  if (wasAPMode) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  } else {
    WiFi.mode(WIFI_STA);
  }
  
  return networksJson;
}

// === 处理根路径请求 ===
void handleRoot() {
  String html = R"=====(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AMS烘干器WiFi设置</title>
  <style>
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
      max-width: 600px; 
      margin: 20px auto; 
      padding: 20px;
      background-color: #f8f9fa;
      line-height: 1.6;
    }
    
    h1, h2 {
      color: #2c3e50;
    }
    
    h1 {
      text-align: center;
      margin-bottom: 25px;
      position: relative;
    }
    
    h1:after {
      content: '';
      display: block;
      width: 80px;
      height: 3px;
      background: #3498db;
      position: absolute;
      bottom: -10px;
      left: 50%;
      transform: translateX(-50%);
      border-radius: 3px;
    }
    
    .wifi-list { 
      list-style: none; 
      padding: 0; 
      margin-top: 20px;
      border-radius: 8px;
      overflow: hidden;
    }
    
    .wifi-item { 
      padding: 12px 15px; 
      border-bottom: 1px solid #eee; 
      cursor: pointer;
      transition: background-color 0.2s;
      display: flex;
      align-items: center;
    }
    
    .wifi-item:hover { 
      background-color: #f5f9ff; 
    }
    
    .wifi-item:last-child {
      border-bottom: none;
    }
    
    .signal-icons {
      display: inline-flex;
      align-items: flex-end;
      height: 14px;
      margin-right: 12px;
    }
    
    .signal-bar {
      width: 3px;
      background-color: #cccccc;
      margin-right: 2px;
      border-radius: 1px 1px 0 0;
    }
    
    .signal-bar.active {
      background-color: #4CAF50;
    }
    
    .signal-bar:nth-child(1) { height: 4px; }
    .signal-bar:nth-child(2) { height: 7px; }
    .signal-bar:nth-child(3) { height: 10px; }
    .signal-bar:nth-child(4) { height: 13px; }
    
    .form-container { 
      margin-top: 30px; 
      padding: 25px; 
      background-color: white; 
      border-radius: 8px; 
      box-shadow: 0 2px 10px rgba(0,0,0,0.05);
    }
    
    .loading { 
      text-align: center; 
      padding: 20px;
      color: #7f8c8d;
    }
    
    .btn { 
      padding: 10px 20px; 
      background-color: #3498db; 
      color: white; 
      border: none; 
      border-radius: 4px; 
      cursor: pointer; 
      font-weight: 600;
      transition: background-color 0.2s;
    }
    
    .btn:hover { 
      background-color: #2980b9; 
    }
    
    #ssid, #password { 
      width: 95%; 
      padding: 10px; 
      margin-bottom: 15px;
      border: 1px solid #ddd;
      border-radius: 4px;
      font-size: 16px;
    }
    
    #ssid:focus, #password:focus {
      border-color: #3498db;
      outline: none;
      box-shadow: 0 0 0 2px rgba(52,152,219,0.2);
    }
    
    .status { 
      margin-top: 15px; 
      padding: 12px; 
      border-radius: 4px; 
      display: none;
      font-size: 14px;
    }
    
    .success { 
      background-color: #d4edda; 
      color: #155724;
      border: 1px solid #c3e6cb;
    }
    
    .error { 
      background-color: #f8d7da; 
      color: #721c24;
      border: 1px solid #f5c6cb;
    }
    
    .restart-note {
      background-color: #fff8e1;
      border-left: 4px solid #ffc107;
      padding: 15px;
      margin-bottom: 25px;
      border-radius: 4px;
    }
    
    .restart-btn {
      margin-top: 10px;
      background-color: #ff9800;
      display: block;
      width: 100%;
      text-align: center;
    }
    
    .restart-btn:hover {
      background-color: #f57c00;
    }
    
    /* 模态框样式 */
    .modal {
      display: none;
      position: fixed;
      z-index: 1000;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0,0,0,0.6);
    }
    
    .modal-content {
      background-color: #fff;
      margin: 15% auto;
      padding: 25px;
      border-radius: 8px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.2);
      width: 85%;
      max-width: 450px;
      position: relative;
    }
    
    .modal-buttons {
      display: flex;
      justify-content: center;
      gap: 15px;
      margin-top: 20px;
    }
    
    .modal-btn {
      padding: 10px 20px;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
      font-weight: 500;
      transition: all 0.2s;
      min-width: 100px;
    }
    
    .confirm-btn {
      background-color: #e74c3c;
      color: white;
    }
    
    .confirm-btn:hover {
      background-color: #c0392b;
    }
    
    .cancel-btn {
      background-color: #95a5a6;
      color: white;
    }
    
    .cancel-btn:hover {
      background-color: #7f8c8d;
    }
    
    .loading-spinner {
      width: 40px;
      height: 40px;
      margin: 20px auto;
      border: 4px solid #f3f3f3;
      border-top: 4px solid #3498db;
      border-radius: 50%;
      animation: spin 1s linear infinite;
    }
    
    @keyframes spin {
      0% { transform: rotate(0deg); }
      100% { transform: rotate(360deg); }
    }
  </style>
</head>
<body>
  <h1>AMS烘干器WiFi设置</h1>
  
  <div class="restart-note">
    <strong>注意：</strong>
    <p>设备只有在启动AP热点模式时才会自动扫描一次附近的WiFi网络，如需刷新列表请重启设备。</p>
    <p>如果列表中没有您的网络，建议直接在下方输入WiFi名称和密码。</p>
    <button class="btn restart-btn" onclick="openRestartModal()">重启烘干器</button>
  </div>
  
  <h2>附近WiFi网络：</h2>
  <div class="loading" id="loading">加载中...</div>
  <ul class="wifi-list" id="wifiList"></ul>
  
  <div class="form-container">
    <form id="wifiForm" onsubmit="return submitForm(event)">
      <label for="ssid">WiFi名称(SSID):</label><br>
      <input type="text" id="ssid" name="ssid" required placeholder="输入WiFi名称"><br>
      
      <label for="password">WiFi密码 (开放网络请留空):</label><br>
      <input type="password" id="password" name="password" placeholder="输入WiFi密码"><br>
      
      <div id="status" class="status"></div>
      
      <button type="submit" class="btn">保存并重启</button>
    </form>
  </div>
  
  <!-- 重启确认模态框 -->
  <div id="restartModal" class="modal">
    <div class="modal-content">
      <h2>确认重启设备</h2>
      <p>设备重启后将会重新扫描附近的WiFi网络，但会中断当前配置过程。</p>
      <p>您确定要立即重启吗？</p>
      <div class="modal-buttons">
        <button class="modal-btn confirm-btn" onclick="confirmRestart()">确认重启</button>
        <button class="modal-btn cancel-btn" onclick="closeRestartModal()">取消</button>
      </div>
    </div>
  </div>

  <script>
    // 页面加载完成后自动加载网络列表
    window.onload = function() {
      loadSavedNetworks();
    };
    
    // 加载已保存的网络列表
    function loadSavedNetworks() {
      const loading = document.getElementById('loading');
      const wifiList = document.getElementById('wifiList');
      
      // 显示加载状态
      loading.style.display = 'block';
      wifiList.innerHTML = '';
      
      // 发送扫描请求
      fetch('/scan')
        .then(response => response.json())
        .then(data => {
          loading.style.display = 'none';
          
          if (data.length === 0) {
            wifiList.innerHTML = '<li class="wifi-item">未找到WiFi网络</li>';
            return;
          }
          
          // 显示扫描结果
          data.forEach(network => {
            const li = document.createElement('li');
            li.className = 'wifi-item';
            
            // 点击效果
            li.onclick = () => {
              // 清除之前选中的效果
              document.querySelectorAll('.wifi-item').forEach(item => {
                item.style.backgroundColor = '';
              });
              
              // 设置当前选中效果
              li.style.backgroundColor = '#e3f2fd';
              
              // 填充SSID
              document.getElementById('ssid').value = network.ssid;
              
              // 滚动到输入框位置
              document.getElementById('wifiForm').scrollIntoView({ 
                behavior: 'smooth', 
                block: 'start' 
              });
              
              // 聚焦到密码输入框
              document.getElementById('password').focus();
            };
            
            // 创建信号强度图标
            const signalIcons = document.createElement('div');
            signalIcons.className = 'signal-icons';
            
            for (let i = 1; i <= 4; i++) {
              const bar = document.createElement('div');
              bar.className = 'signal-bar';
              if (i <= network.strength) bar.classList.add('active');
              signalIcons.appendChild(bar);
            }
            
            li.appendChild(signalIcons);
            li.appendChild(document.createTextNode(network.ssid));
            wifiList.appendChild(li);
          });
        })
        .catch(error => {
          console.error('扫描错误:', error);
          loading.style.display = 'none';
          wifiList.innerHTML = '<li class="wifi-item">扫描网络时出错，请重试</li>';
        });
    }
    
    // 打开重启模态框
    function openRestartModal() {
      document.getElementById('restartModal').style.display = 'block';
    }
    
    // 关闭重启模态框
    function closeRestartModal() {
      document.getElementById('restartModal').style.display = 'none';
    }
    
    // 确认重启
    function confirmRestart() {
      document.querySelector('.modal-content').innerHTML = `
        <h2>设备重启中。。。</h2>
        <p>请稍候，设备将在5秒后重启</p>
        <div class="loading-spinner"></div>
      `;
      
      // 发送重启请求
      fetch('/reboot', { method: 'POST' })
      .then(response => response.text())
      .then(data => {
        document.querySelector('.modal-content').innerHTML = `
          <h2>重启成功</h2>
          <p>${data}</p>
        `;
      })
      .catch(error => {
        document.querySelector('.modal-content').innerHTML = `
          <h2>重启失败</h2>
          <p>${error}</p>
        `;
      });
    }

    // 表单提交处理
    function submitForm(event) {
      event.preventDefault();
      
      const ssid = document.getElementById('ssid').value;
      const password = document.getElementById('password').value;
      const statusDiv = document.getElementById('status');
      
      statusDiv.style.display = 'block';
      statusDiv.className = 'status';
      statusDiv.textContent = '正在保存配置...';
      
      // 输入验证
      if (ssid.length < 1 || ssid.length > 60) {
        statusDiv.textContent = '错误：WiFi名称长度应在1-60个字符之间';
        statusDiv.classList.add('error');
        return false;
      }
      
      // 发送数据到服务器
      fetch('/save', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: 'ssid=' + encodeURIComponent(ssid) + '&password=' + encodeURIComponent(password)
      })
      .then(response => {
        if (!response.ok) {
          throw new Error('服务器响应错误');
        }
        return response.text();
      })
      .then(data => {
        statusDiv.textContent = '配置已保存，设备将在5秒后重启...';
        statusDiv.classList.add('success');
        
        // 显示倒计时
        let count = 5;
        const countdown = setInterval(() => {
          statusDiv.textContent = '配置已保存，设备将在' + count + '秒后重启...';
          count--;
          
          if (count < 0) {
            clearInterval(countdown);
            statusDiv.textContent = '设备正在重启，稍后请用同局域网设备访问屏幕上的IP';
          }
        }, 1000);
      })
      .catch(error => {
        console.error('保存错误:', error);
        statusDiv.textContent = '保存配置时出错: ' + error.message;
        statusDiv.classList.add('error');
      });
      
      return false;
    }
  </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
}

// === 重启处理函数 ===
void handleReboot() {
  server.send(200, "text/plain", "设备正在重启，稍后请用同局域网设备访问屏幕上的IP");
  delay(3000);
  ESP.restart();
}

// === 处理配置保存请求 ===
void handleSaveConfig() {
  // 获取表单数据
  String newSSID = server.arg("ssid");
  String newPassword = server.arg("password");
  
  // 保存到Preferences
  preferences.putString("wifi_ssid", newSSID);
  preferences.putString("wifi_password", newPassword);
  
  // 返回响应
  String message = "<html><body>Configuration saved. Device will restart in 5 seconds...</body></html>";
  server.send(200, "text/html", message);
  
  // 重启
  delay(5000);
  preferences.end();
  ESP.restart();
}

void setupMQTT() {
  String savedSERVER = "";
  int savedPORT = 0;
  String savedUSER = "";
  String savedPASS = "";

  if (preferences.isKey("mqtt_server")) {
    savedSERVER = preferences.getString("mqtt_server", "");
  }
  if (preferences.isKey("mqtt_port")) {
    savedPORT = preferences.getInt("mqtt_port", 1883);
  }
  if (preferences.isKey("mqtt_user")) {
    savedUSER = preferences.getString("mqtt_user", "");
  }
  if (preferences.isKey("mqtt_pass")) {
    savedPASS = preferences.getString("mqtt_pass", "");
  }
  if (savedSERVER != "") {
    strncpy(mqtt_server, savedSERVER.c_str(), sizeof(mqtt_server) - 1);
    mqtt_server[sizeof(mqtt_server) - 1] = '\0';
  }
  if (savedPORT != 0) {
    mqtt_port = savedPORT;
  }
  if (savedUSER != "") {
    strncpy(mqtt_user, savedUSER.c_str(), sizeof(mqtt_user) - 1);
    mqtt_user[sizeof(mqtt_user) - 1] = '\0';
  }
  if (savedPASS != "") {
    strncpy(mqtt_pass, savedPASS.c_str(), sizeof(mqtt_pass) - 1);
    mqtt_pass[sizeof(mqtt_pass) - 1] = '\0';
  }

  // 是否禁用
  if (strcmp(mqtt_server, "0.0.0.0") == 0) {
    mqttEnabled = false;
    Serial.println("MQTT disabled (0.0.0.0)");
    display.clearDisplay();
    drawZh(24,12,mqtt_server_80x20,display);
    drawZh(34,32,is_disable_60x20,display);
    display.display();
    delay(1000);
    return;
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(handleMQTT);

  if (mqttEnabled) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
  }
}

void reconnectMQTT() {
  if (!mqttEnabled || WiFi.status() != WL_CONNECTED) return;
  unsigned long currentMillis = millis();
  
  // 检查是否到了重连时间
  if (currentMillis - lastReconnectAttempt >= RECONNECT_INTERVAL) {
    lastReconnectAttempt = currentMillis;
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("DryerController", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      
      // 订阅控制主题
      mqttClient.subscribe(TOPIC_SET_TEMP);
      mqttClient.subscribe(TOPIC_SET_TIME);
      mqttClient.subscribe(TOPIC_SET_MODE);
      mqttClient.subscribe(TOPIC_POWER);
      mqttClient.subscribe(TOPIC_STOP);
      mqttClient.subscribe(TOPIC_AUTOTUNE);
      mqttClient.subscribe(TOPIC_SET_FAN);
      mqttClient.subscribe(TOPIC_SET_EXHAUST);
      mqttClient.subscribe(TOPIC_SET_EXHAUST_TIME);
      mqttClient.subscribe(TOPIC_SET_EXHAUST_MODE);
      mqttClient.subscribe(TOPIC_SET_EXHAUST_FIRST_DELAY);
      mqttClient.subscribe(TOPIC_SET_EXHAUST_ON_DURATION);
      mqttClient.subscribe(TOPIC_SET_EXHAUST_OFF_DURATION);
      mqttClient.subscribe(TOPIC_SET_AUTO_DRY_ENABLE);
      mqttClient.subscribe(TOPIC_SET_AUTO_DRY_START);
      mqttClient.subscribe(TOPIC_SET_AUTO_DRY_DURATION);
      mqttClient.subscribe(TOPIC_SET_AUTO_DRY_COOLDOWN);
      mqttClient.publish(TOPIC_STATE_IP, WiFi.localIP().toString().c_str());

      // 清除错误显示
      lastErrorMessage = "";
      
      publishState(); // 连接后立即发布状态
      
      // 发布性能监控数据
      publishCurrentPerformanceData();
    } else {
      // 连接失败时显示错误
      String errorMsg = "MQTT connect failed";
      String ch_msg = "MQTT服务器连接失败" + String(mqttClient.state());
      setErrorMessage(errorMsg,ch_msg);
      
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(", will try again later");
    }
  }
}

void readTemperature() {
  sensors.requestTemperatures();
  currentTemp = sensors.getTempCByIndex(0);
  chamberSensor.requestTemperatures();
  double rawChamberTemp = chamberSensor.getTempCByIndex(0);

  // 仓温传感器
  if (rawChamberTemp != DEVICE_DISCONNECTED_C && 
      !isnan(rawChamberTemp) &&
      rawChamberTemp > -50 && 
      rawChamberTemp < 200) {
    chamberTemp = rawChamberTemp;
    chamberSensorConnected = true;
  } else {
    chamberTemp = NAN;
    chamberSensorConnected = false;
  }

  // AHT30温湿度数据
  if (aht30Connected) {
    aht.getEvent(&aht_humidity, &aht_temp);
    aht30Temperature = aht_temp.temperature;
    aht30Humidity = aht_humidity.relative_humidity;
  }

  // 记录历史数据
  unsigned long currentTime = millis();
  if (currentTime - lastHistoryUpdate_web >= 1000) { // 每秒记录一次
    tempHistory_web[historyIndex_web].mainTemp_web = currentTemp;
    tempHistory_web[historyIndex_web].chamberTemp_web = chamberTemp;
    tempHistory_web[historyIndex_web].timestamp_web = currentTime;
    historyIndex_web = (historyIndex_web + 1) % HISTORY_SIZE_web;
    lastHistoryUpdate_web = currentTime;
  }

  // 主传感器错误处理
  if (currentTemp == DEVICE_DISCONNECTED_C || isnan(currentTemp) ||
        currentTemp < -50 || currentTemp > 125) {
    String errorMsg = "Temp Sensor Error!";
    String ch_msg = "出风口温度传感器异常";
    Serial.println(errorMsg);
    setErrorMessage(errorMsg,ch_msg);
    stopDrying();
    mqttClient.publish(TOPIC_ALERTS, ch_msg.c_str());
    return;
  }else{
    if(lastErrorMessage == "Temp Sensor Error!"){
      clearErrorMessage();
    }
  }
}

void updateDisplay() {
  display.clearDisplay();

  // 显示错误信息
  if (millis() - errorDisplayTime < ERROR_DISPLAY_DURATION) {
    drawErrorMessage();
    return;
  }

  // 异常网络状态
  if (WiFi.status() != WL_CONNECTED) {
    drawZh(81,31,wifi_disconnected_20,display);
  } else if (!mqttClient.connected() && mqttEnabled) {
    drawZh(48,19,mqtt_error_32,display);
  }

  // 显示工作状态
  switch(currentState) {
    case STANDBY: drawZh(111,17,standby_15x30,display);; break;
    case DRYING: drawZh(111,10,running_15x45,display);; break;
  }

  // 第一行
  drawZh(0,0,temp_30x13,display);
  display.setCursor(33, 4);
  display.print(currentTemp, 1);
  display.print("/");
  display.print((int)targetTemp);
  display.print("C");

  // 第二行
  drawZh(0,14,time_30x13,display);
  display.setCursor(33, 19);
  if (currentState == DRYING) {
    if (remainingTime == -1234) {
      drawZh(33,14,continuous_running_60x13,display);
    }else{
      int mins = remainingTime / 60;
      int secs = remainingTime % 60;
      display.print(mins);
      display.print(":");
      if (secs < 10) display.print("0");
      display.print(secs);
      display.print("/");
      display.print(targetTime);
      display.print("m");
    }
  } else {
    display.print(targetTime);
    display.print("min");
  }

  // 第三行
  drawZh(0,28,mode_30x13,display);
  if (currentPreset == 0) {
    drawZh(33,28,custom_39x13,display);
  } else {
    display.setCursor(33, 32);
    display.print(PRESETS[currentPreset-1].name);
  }

  // 第四行
  drawZh(0,41,cangtemp_30x13,display);
  if (chamberSensorConnected) {
    display.setCursor(33, 45);
    display.print(chamberTemp, 1);
    display.print("C");
  }else{
   drawZh(33,41,cang_temp_noconnect_39x13,display);
  }

  // 第五行
  display.setCursor(0, 55);
  if (apMode) {
      display.print("AP Mode: " + apSSID);
  } else {
      display.print("IP: ");
      display.print(WiFi.localIP());
  }

  display.display();
}

void startDrying() {
  if (targetTime < 0) return;
  currentState = DRYING;
  safetyShutdown = false;
  dryingStartTime = millis();
  exhaustCycleActive = exhaustAutoMode;
  exhaustCycleState = false; // 初始关闭排气阀
  exhaustFirstOpenDone = false;
  lastExhaustCycleTime = dryingStartTime;
  if (targetTime == 0) {
    remainingTime = -1234; // 永久运行
  } else {
    remainingTime = targetTime * 60; // 转换为秒
  }
  heaterActive = true;
  fanRunning = true;

  // 重置PID状态，避免上一次积分残留
  pidSetpointTemp = currentTemp;
  pidInputTemp = currentTemp;
  lastPidUpdate = millis();
  heaterPID.SetMode(MANUAL);
  heaterPID.SetMode(AUTOMATIC);

  // 打开风扇
  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);

  publishState();
}

void stopDrying(bool userStop) {
  currentState = STANDBY;
  heaterActive = false;
  safetyShutdown = false;
  ledcWrite(HEATER_PWM_CHANNEL, 0); // 关闭加热器
  exhaustCycleActive = false;
  exhaustFirstOpenDone = false;
  closeExhaustValve();
  remainingTime = 0;

  // 用户在自动烘干过程中主动停止时，启动冷却期，防止短时间内再次自动触发
  if (userStop && autoDryRunning && autoDryCooldownMinutes > 0) {
    autoDryBlockUntil = millis() + (unsigned long)autoDryCooldownMinutes * 60000UL;
  }

  publishState();
}

void updateSystemState() {
  if (heaterActive && !safetyShutdown) {
    unsigned long now = millis();
    double dtSec = (lastPidUpdate == 0) ? 0.001 : (now - lastPidUpdate) / 1000.0;
    lastPidUpdate = now;

    // 传感器滤波，降低抖动对PID的影响
    pidInputTemp = filteredTemperature(currentTemp);

    // 设定值斜坡，避免一步到位造成过冲
    double delta = targetTemp - pidSetpointTemp;
    double maxStep = PID_SETPOINT_RAMP_RATE * dtSec;
    if (fabs(delta) > maxStep) {
      pidSetpointTemp += (delta > 0 ? maxStep : -maxStep);
    } else {
      pidSetpointTemp = targetTemp;
    }

    double absError = fabs(pidSetpointTemp - pidInputTemp);
    double activeKi = (absError > PID_I_ACTIVE_BAND) ? (Ki * PID_I_REDUCED_FACTOR) : Ki;

    // 根据误差动态调整积分与输出限制，接近目标时限制最大功率
    heaterPID.SetTunings(Kp, activeKi, Kd);
    double maxOutput = absError < PID_NEAR_SETPOINT_BAND ? PID_NEAR_SETPOINT_MAX_OUTPUT : 255;
    heaterPID.SetOutputLimits(MIN_PWM, maxOutput);

    heaterPID.Compute();

    int pwm = (int)pidOutput;
    pwm = constrain(pwm, (int)MIN_PWM, (int)maxOutput);

    ledcWrite(HEATER_PWM_CHANNEL, pwm);
  } else if (safetyShutdown) {
    ledcWrite(HEATER_PWM_CHANNEL, 0);
  } else {
    ledcWrite(HEATER_PWM_CHANNEL, 0); // 正常关闭加热器
    pidSetpointTemp = targetTemp;
  }

  controlFan();
}

void controlFan() {

  // 全局风扇控制：温度 > 40℃ 就启动风扇
  if (currentTemp > 40.0) {
    fanRunning = true;
    ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
  }
  // 温度 <= 40℃ 关闭风扇（但烘干状态除外）
  else if (currentTemp <= 40.0 && !(currentState == DRYING || autotuneState != ATUNE_OFF)) {
    if (fanRunning) {
      fanRunning = false;
      ledcWrite(FAN_PWM_CHANNEL, 0);
    }
  }

  // 烘干状态：即使温度低于40℃也保持风扇运行
  if (currentState == DRYING) {
    fanRunning = true;
    ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
  }
}

void handleMQTT(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  // 设置温度
  if (strcmp(topic, TOPIC_SET_TEMP) == 0) {
    int newTemp = atoi(message);
    newTemp = (newTemp / 5) * 5;
    if (newTemp < MIN_TEMP) newTemp = MIN_TEMP;
    if (newTemp > MAX_TEMP) newTemp = MAX_TEMP;
    if (autotuneState == ATUNE_OFF) {
      targetTemp = (double)newTemp;
      currentPreset = 0;
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putDouble("targetTemp", targetTemp);
    }
    publishState();
  }

  // 设置时间
  else if (strcmp(topic, TOPIC_SET_TIME) == 0) {
    int newTime = atoi(message);
    newTime = (newTime / 30) * 30;
    if (newTime < 0) newTime = 0;
    if (newTime > 1440) newTime = 1440;
    if (autotuneState == ATUNE_OFF) {
      targetTime = newTime;
      currentPreset = 0;
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putInt("targetTime", targetTime);
    }
    publishState();
  }

  // 设置模式
  else if (strcmp(topic, TOPIC_SET_MODE) == 0) {
    String mode = String(message);
    if (autotuneState == ATUNE_OFF) {
      if (mode == "CUSTOM") {
        currentPreset = 0;
        preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      } else {
        for (int i = 0; i < PRESET_COUNT; i++) {
          if (mode == PRESETS[i].name) {
            currentPreset = i + 1;
            preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
            targetTemp = (double)PRESETS[i].temp;
            targetTime = PRESETS[i].time;
            preferences.putDouble("targetTemp", targetTemp);
            preferences.putInt("targetTime", targetTime);
            break;
          }
        }
      }
      if (mode == "CUSTOM") {
        preferences.putDouble("targetTemp", targetTemp);
        preferences.putInt("targetTime", targetTime);
      }
    }
    publishState();
  }

  // 设置风速
  else if (strcmp(topic, TOPIC_SET_FAN) == 0) {
    int percent = atoi(message);
    int ln_speed = (percent*255)/100;
    if(ln_speed<FAN_MIN_PWM)ln_speed=FAN_MIN_PWM;
    else if(ln_speed>255)ln_speed=255;
    fanSpeed = ln_speed;
    preferences.putInt("fanSpeed", fanSpeed);
    controlFan();
    publishState();
  }

  // 排气阀控制：OPEN/CLOSE/TOGGLE_DIRECTION
  else if (strcmp(topic, TOPIC_SET_EXHAUST) == 0) {
    String cmd = String(message);
    cmd.toUpperCase();
    if (cmd == "OPEN") {
      openExhaustValve();
    } else if (cmd == "CLOSE") {
      closeExhaustValve();
    } else if (cmd == "TOGGLE" || cmd == "TOGGLE_DIRECTION") {
      toggleExhaustMotorDirection();
    }
    publishState();
  }

  // 排气阀动作时间 (ms)
  else if (strcmp(topic, TOPIC_SET_EXHAUST_TIME) == 0) {
    int ms = atoi(message);
    setExhaustActionTime((unsigned long)ms);
    publishState();
  }

  // 排气循环时间（分钟）
  else if (strcmp(topic, TOPIC_SET_EXHAUST_FIRST_DELAY) == 0) {
    unsigned long minutes = strtoul(message, nullptr, 10);
    setExhaustFirstDelayMinutes(minutes);
    publishState();
  }
  else if (strcmp(topic, TOPIC_SET_EXHAUST_ON_DURATION) == 0) {
    unsigned long minutes = strtoul(message, nullptr, 10);
    setExhaustOnDurationMinutes(minutes);
    publishState();
  }
  else if (strcmp(topic, TOPIC_SET_EXHAUST_OFF_DURATION) == 0) {
    unsigned long minutes = strtoul(message, nullptr, 10);
    setExhaustOffDurationMinutes(minutes);
    publishState();
  }

  // 自动烘干(湿度)配置
  else if (strcmp(topic, TOPIC_SET_AUTO_DRY_ENABLE) == 0) {
    String flag = String(message);
    flag.toUpperCase();
    if (flag == "ON" || flag == "TRUE" || flag == "1") {
      setAutoDryEnabled(true);
    } else if (flag == "OFF" || flag == "FALSE" || flag == "0") {
      setAutoDryEnabled(false);
    }
    publishState();
  }
  else if (strcmp(topic, TOPIC_SET_AUTO_DRY_START) == 0) {
    setAutoDryStart(String(message).toFloat());
    publishState();
  }
  else if (strcmp(topic, TOPIC_SET_AUTO_DRY_DURATION) == 0) {
    setAutoDryRunMinutes(String(message).toInt());
    publishState();
  }
  else if (strcmp(topic, TOPIC_SET_AUTO_DRY_COOLDOWN) == 0) {
    setAutoDryCooldownMinutes(String(message).toInt());
    publishState();
  }

  // 排气阀模式 AUTO/MANUAL
  else if (strcmp(topic, TOPIC_SET_EXHAUST_MODE) == 0) {
    String mode = String(message);
    mode.toUpperCase();
    if (mode == "AUTO") {
      exhaustAutoMode = true;
    } else if (mode == "MANUAL") {
      exhaustAutoMode = false;
    }
    preferences.putBool("exhaustAutoMode", exhaustAutoMode);

    // 与手动/自动模式保持一致的循环控制
    if (!exhaustAutoMode) {
      exhaustCycleActive = false;
    } else if (currentState == DRYING) {
      exhaustCycleActive = true;
      exhaustCycleState = false;
      lastExhaustCycleTime = millis();
    }
    publishState();
  }

  // 开始
  else if (strcmp(topic, TOPIC_POWER) == 0) {
    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      startDrying();
    }
  }

  // 停止
  else if (strcmp(topic, TOPIC_STOP) == 0) {
    if (currentState == DRYING) {
      stopDrying(true);
    }
    else if (autotuneState != ATUNE_OFF) {
      autotuneState = ATUNE_OFF;
      heaterActive = false;
      ledcWrite(HEATER_PWM_CHANNEL, 0);
      heaterPID.SetTunings(Kp, Ki, Kd);
      currentState = STANDBY;
      publishState();
    }
  }

  // 自动PID整定
  else if (strcmp(topic, TOPIC_AUTOTUNE) == 0) {
    if (strcmp(message, "START") == 0) {
      if (autotuneState == ATUNE_OFF && currentState == STANDBY) {
        startAutoTune();
      }
    }
  }
}

void publishState() {
  if (!mqttEnabled || !mqttClient.connected()) return;
  // 发布主温度
  char tempStr[10];
  dtostrf(currentTemp, 4, 1, tempStr);
  mqttClient.publish(TOPIC_STATE_TEMP, tempStr);

  // 发布仓温
  if (chamberSensorConnected) {
    char chamberTempStr[16];
    dtostrf(chamberTemp, 4, 1, chamberTempStr);
    mqttClient.publish(TOPIC_STATE_CHAMBER_TEMP, chamberTempStr);
  }else{
    mqttClient.publish(TOPIC_STATE_CHAMBER_TEMP, "-1234");
  }

  // 发布AHT30温湿度数据
  if (aht30Connected) {
    char aht30TempStr[10];
    char aht30HumidityStr[10];
    dtostrf(aht30Temperature, 4, 1, aht30TempStr);
    dtostrf(aht30Humidity, 4, 1, aht30HumidityStr);
    mqttClient.publish(TOPIC_STATE_AHT30_TEMP, aht30TempStr);
    mqttClient.publish(TOPIC_STATE_AHT30_HUMIDITY, aht30HumidityStr);
  }else{
    mqttClient.publish(TOPIC_STATE_AHT30_TEMP, "-1234");
    mqttClient.publish(TOPIC_STATE_AHT30_HUMIDITY, "-1234");
  }

  // 目标温度 - 发布为整数
  char targetTempStr[6];
  itoa((int)targetTemp, targetTempStr, 10);
  mqttClient.publish(TOPIC_STATE_TARGET_TEMP, targetTempStr);

  // 设置时间
  char timeStr[6];
  itoa(targetTime, timeStr, 10);
  mqttClient.publish(TOPIC_STATE_TIME, timeStr);

  // 剩余时间(秒)
  if (remainingTime == -1234) {
    mqttClient.publish(TOPIC_STATE_REMAINING, "-1234");
  } else {
    char remainingStr[16];
    itoa(remainingTime, remainingStr, 10);
    mqttClient.publish(TOPIC_STATE_REMAINING, remainingStr);
  }

  // 模式
  const char* mode = (currentPreset == 0) ? "CUSTOM" : PRESETS[currentPreset-1].name;
  mqttClient.publish(TOPIC_STATE_MODE, mode);

  // 风速
  char fanStr[6];
  int ln_speed = fanSpeed;
  if(ln_speed<FAN_MIN_PWM) ln_speed=FAN_MIN_PWM;
  else if(ln_speed>255) ln_speed=255;
  ln_speed = (ln_speed*100)/255;
  itoa(ln_speed, fanStr, 10);
  mqttClient.publish(TOPIC_STATE_FAN, fanStr);

  // 排气阀状态
  mqttClient.publish(TOPIC_STATE_EXHAUST, getExhaustStateLabel().c_str());
  mqttClient.publish(TOPIC_STATE_EXHAUST_MODE, exhaustAutoMode ? "AUTO" : "MANUAL");
  char exhaustTimeStr[12];
  ultoa(exhaustActionTime, exhaustTimeStr, 10);
  mqttClient.publish(TOPIC_STATE_EXHAUST_TIME, exhaustTimeStr);
  mqttClient.publish(TOPIC_STATE_EXHAUST_DIRECTION, exhaustMotorReversed ? "反向" : "正向");
  char exhaustFirstDelayStr[12];
  ultoa(exhaustFirstDelay / 60000UL, exhaustFirstDelayStr, 10);
  mqttClient.publish(TOPIC_STATE_EXHAUST_FIRST_DELAY, exhaustFirstDelayStr);
  char exhaustOnDurationStr[12];
  ultoa(exhaustOnDuration / 60000UL, exhaustOnDurationStr, 10);
  mqttClient.publish(TOPIC_STATE_EXHAUST_ON_DURATION, exhaustOnDurationStr);
  char exhaustOffDurationStr[12];
  ultoa(exhaustOffDuration / 60000UL, exhaustOffDurationStr, 10);
  mqttClient.publish(TOPIC_STATE_EXHAUST_OFF_DURATION, exhaustOffDurationStr);

  // 自动烘干(湿度)状态
  mqttClient.publish(TOPIC_STATE_AUTO_DRY_ENABLE, autoDryEnabled ? "ON" : "OFF");
  char autoDryStartStr[12];
  dtostrf(autoDryStartHumidity, 4, 1, autoDryStartStr);
  mqttClient.publish(TOPIC_STATE_AUTO_DRY_START, autoDryStartStr);
  char autoDryRunStr[12];
  ultoa((unsigned long)autoDryRunMinutes, autoDryRunStr, 10);
  mqttClient.publish(TOPIC_STATE_AUTO_DRY_DURATION, autoDryRunStr);
  char autoDryCooldownStr[12];
  ultoa((unsigned long)autoDryCooldownMinutes, autoDryCooldownStr, 10);
  mqttClient.publish(TOPIC_STATE_AUTO_DRY_COOLDOWN, autoDryCooldownStr);

  // 状态
  const char* status;
  switch(currentState) {
    case STANDBY: status = "待机"; break;
    case DRYING: status = "烘干中"; break;
    case PID_AUTOTUNE: status = "PID自动整定";break;
    default: status = "unknown";
  }
  mqttClient.publish(TOPIC_STATE_STATUS, status);

  // PID参数
  char pidStr[30];
  snprintf(pidStr, sizeof(pidStr), "%.2f-%.4f-%.2f", Kp, Ki, Kd);
  mqttClient.publish(TOPIC_STATE_PID, pidStr);

  // IP
  mqttClient.publish(TOPIC_STATE_IP, WiFi.localIP().toString().c_str());

  // 清除警报
  if(lastErrorMessage == ""){
    mqttClient.publish(TOPIC_ALERTS, "无");
  }
}

void safetyCheck() {
  // 温度保护：超过设定温度5度暂停加热
  if (heaterActive && currentTemp > (targetTemp + SAFETY_OVERTEMP) && !safetyShutdown) {
    String errorMsg = "Overtemp! Heating paused";
    String ch_msg = "超过设定温度过高，暂停加热，等待降温后继续运行";
    Serial.println("Safety: " + errorMsg);
    setErrorMessage(errorMsg, ch_msg);
    ledcWrite(HEATER_PWM_CHANNEL, 0);
    safetyShutdown = true;
    safetyResumeTemp = targetTemp;
    fanRunning = true;
    ledcWrite(FAN_PWM_CHANNEL, 255);
    mqttClient.publish(TOPIC_ALERTS, ch_msg.c_str());
  }
  
  // 检查是否可以恢复加热
  if (safetyShutdown && currentTemp <= safetyResumeTemp) {
    safetyShutdown = false;
    String resumeMsg = "Heating resumed";
    Serial.println("Safety: " + resumeMsg);
    heaterActive = true;
    if(lastErrorMessage == "Overtemp! Heating paused"){
      clearErrorMessage();
    }
  }
  
  // 在安全保护期间，确保加热器保持关闭
  if (safetyShutdown) {
    ledcWrite(HEATER_PWM_CHANNEL, 0);
  }
}

void setErrorMessage(String message, String ch_message) {
  lastErrorMessage = message;
  lastErrorMessage_ch = ch_message;
  errorDisplayTime = millis();
  drawErrorMessage();
}

void clearErrorMessage() {
  lastErrorMessage = "";
  lastErrorMessage_ch = "";
}

void drawErrorMessage() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ERROR:");
  if(lastErrorMessage == "Autotune: Sensor Fail"){
    drawZh(8,10,PID_auto_fail_112x16,display);
    drawZh(24,27,out_temp_error_80x32,display);
  }else if(lastErrorMessage == "Autotune Timeout"){
    drawZh(8,10,PID_auto_timeout_112x16,display);
  }else if(lastErrorMessage == "Autotune: Not enough cycles"){
    drawZh(8,10,PID_auto_fail_112x16,display);
    drawZh(16,27,no_enough_cycle_96x16,display);
  }else if(lastErrorMessage == "MQTT connect failed"){
    drawZh(24,10,failed_MQTT_connect_80x32,display);
  }else if(lastErrorMessage == "Temp Sensor Error!"){
    drawZh(24,10,out_temp_error_80x32,display);
  }else if(lastErrorMessage == "Overtemp! Shutdown"){
    drawZh(16,10,overtemp_96x32,display);
  }
  else{
    display.setCursor(0, 10);
    display.print(lastErrorMessage);
  }
  display.display();
}

// === 专门用于STA模式的WiFi扫描函数 ===
void staScanNetworks() {
  // 保存当前WiFi状态
  bool wasConnected = (WiFi.status() == WL_CONNECTED);
  String currentSSID = wasConnected ? WiFi.SSID() : "";
  String currentPass = wasConnected ? WiFi.psk() : "";
  
  // 开始扫描
  Serial.println("[STA] Starting WiFi scan...");
  int numNetworks = WiFi.scanNetworks();
  
  // 构建JSON结果
  String networksJson = "[";
  for (int i = 0; i < numNetworks; i++) {
    String ssid = WiFi.SSID(i);
    
    // 转义JSON特殊字符
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    
    // 计算信号强度（0-4）
    int rssi = WiFi.RSSI(i);
    int strength = 0;
    if (rssi > -55) strength = 4;      // 强
    else if (rssi > -65) strength = 3; // 良好
    else if (rssi > -75) strength = 2; // 中等
    else if (rssi > -85) strength = 1; // 弱
    
    // 添加到JSON数组
    networksJson += "{\"ssid\":\"" + ssid + "\",\"strength\":" + String(strength) + "}";
    if (i < numNetworks - 1) networksJson += ",";
  }
  networksJson += "]";
  
  Serial.println("[STA] Scan completed: " + String(numNetworks) + " networks found");
  
  // 清理扫描结果
  WiFi.scanDelete();
  
  // 恢复之前的连接
  if (wasConnected) {
    Serial.println("[STA] Reconnecting to original network: " + currentSSID);
    WiFi.begin(currentSSID.c_str(), currentPass.c_str());
    
    // 等待重新连接
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
      delay(100);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[STA] Reconnected successfully");
    } else {
      Serial.println("[STA] Reconnection failed");
    }
  }
  
  staScanResult = networksJson;
}
// === 实现WiFi配置页面 ===
void handleWifiConfigPage() {
  String html = R"=====(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WiFi配置</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    
    body {
      background: linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%);
      min-height: 100vh;
      padding: 20px;
      color: #2c3e50;
    }
    
    .container {
      max-width: 600px;
      margin: 0 auto;
      background: white;
      border-radius: 15px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.1);
      overflow: hidden;
      position: relative;
    }
    
    .back-btn {
      position: absolute;
      top: 15px;
      left: 15px;
      background: #3498db;
      color: white;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.2rem;
      text-decoration: none;
      z-index: 10;
      transition: all 0.3s;
    }
    
    .back-btn:hover {
      background: #2980b9;
      transform: translateY(-2px);
      box-shadow: 0 3px 8px rgba(0,0,0,0.1);
    }
    
    header {
      background: #3498db;
      color: white;
      padding: 55px 30px 25px;
      text-align: center;
      position: relative;
    }
    
    h1 {
      font-size: 2.0rem;
      margin-bottom: 5px;
      font-weight: 600;
    }
    
    .content {
      padding: 25px;
    }
    
    .section {
      margin-bottom: 30px;
      padding: 20px;
      background: #f8f9fa;
      border-radius: 10px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.05);
    }
    
    .section-title {
      font-size: 1.4rem;
      margin-bottom: 15px;
      padding-bottom: 10px;
      border-bottom: 2px solid #3498db;
      color: #2c3e50;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    
    .refresh-btn {
      background: #3498db;
      color: white;
      border: none;
      border-radius: 4px;
      padding: 8px 15px;
      font-size: 1rem;
      cursor: pointer;
      transition: all 0.3s;
    }
    
    .refresh-btn:hover {
      background: #2980b9;
      transform: translateY(-2px);
    }
    
    .wifi-list {
      list-style: none;
      margin-top: 15px;
      max-height: 300px;
      overflow-y: auto;
      border: 1px solid #e9ecef;
      border-radius: 8px;
    }
    
    .wifi-item {
      padding: 12px 15px;
      border-bottom: 1px solid #eee;
      cursor: pointer;
      transition: background-color 0.2s;
      display: flex;
      align-items: center;
    }
    
    .wifi-item:hover {
      background-color: #f0f7ff;
    }
    
    .wifi-item:last-child {
      border-bottom: none;
    }
    
    .signal-icons {
      display: inline-flex;
      align-items: flex-end;
      height: 20px;
      margin-right: 12px;
    }
    
    .signal-bar {
      width: 4px;
      margin-right: 3px;
      border-radius: 2px 2px 0 0;
      background-color: #ddd;
    }
    
    .signal-bar.active {
      background-color: #4CAF50;
    }
    
    .signal-bar:nth-child(1) { height: 5px; }
    .signal-bar:nth-child(2) { height: 10px; }
    .signal-bar:nth-child(3) { height: 15px; }
    .signal-bar:nth-child(4) { height: 20px; }
    
    .form-group {
      margin-bottom: 20px;
    }
    
    label {
      display: block;
      font-size: 1.1rem;
      margin-bottom: 8px;
      font-weight: 500;
      color: #495057;
    }
    
    input {
      width: 100%;
      padding: 14px;
      font-size: 1.1rem;
      border: 1px solid #dee2e6;
      border-radius: 8px;
      transition: all 0.3s;
    }
    
    input:focus {
      outline: none;
      border-color: #3498db;
      box-shadow: 0 0 0 3px rgba(52, 152, 219, 0.2);
    }
    
    .btn-submit {
      display: block;
      width: 100%;
      padding: 16px;
      background: #2ecc71;
      color: white;
      font-size: 1.2rem;
      font-weight: 600;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.3s;
      margin-top: 15px;
    }
    
    .btn-submit:hover {
      background: #27ae60;
      transform: translateY(-3px);
      box-shadow: 0 5px 15px rgba(0,0,0,0.1);
    }
    
    .status {
      margin-top: 15px;
      padding: 15px;
      border-radius: 8px;
      display: none;
      text-align: center;
      font-size: 1.1rem;
    }
    
    .success {
      background: #d4edda;
      color: #155724;
      border: 1px solid #c3e6cb;
    }
    
    .error {
      background: #f8d7da;
      color: #721c24;
      border: 1px solid #f5c6cb;
    }
    
    .current-info {
      background: #e3f2fd;
      padding: 15px;
      border-radius: 8px;
      margin-bottom: 20px;
      font-size: 1.1rem;
    }
    
    .info-row {
      margin-bottom: 8px;
      display: flex;
    }
    
    .info-label {
      font-weight: 600;
      width: 120px;
    }
    
    .loading {
      text-align: center;
      padding: 30px;
      color: #7f8c8d;
      display: none;
    }
    
    .loading-spinner {
      width: 40px;
      height: 40px;
      margin: 0 auto 15px;
      border: 4px solid #f3f3f3;
      border-top: 4px solid #3498db;
      border-radius: 50%;
      animation: spin 1s linear infinite;
    }
    
    @keyframes spin {
      0% { transform: rotate(0deg); }
      100% { transform: rotate(360deg); }
    }
    
    @media (max-width: 480px) {
      .content {
        padding: 15px;
      }
      
      header {
        padding: 60px 20px 20px;
      }
      
      .section {
        padding: 15px;
      }
      
      .section-title {
        font-size: 1.2rem;
        flex-direction: column;
        align-items: flex-start;
      }
      
      .refresh-btn {
        margin-top: 10px;
        align-self: flex-end;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <a href="/" class="back-btn">
      ←
    </a>
    
    <header>
      <h1>WiFi网络配置</h1>
    </header>
    
    <div class="content">
      <div class="current-info">
        <div class="info-row">
          <span class="info-label">当前网络:</span>
          <span id="currentSSID">)=====";
  
  // 添加当前WiFi信息
  if (WiFi.status() == WL_CONNECTED) {
    html += WiFi.SSID();
  } else {
    html += "未连接";
  }
  
  html += R"=====(</span>
        </div>
        <div class="info-row">
          <span class="info-label">IP地址:</span>
          <span id="currentIP">)=====";
          
  if (WiFi.status() == WL_CONNECTED) {
    html += WiFi.localIP().toString();
  } else {
    html += "N/A";
  }
  
  html += R"=====(</span>
        </div>
        <div class="info-row">
          <span class="info-label">信号强度:</span>
          <span id="currentStrength">)=====";
  
  if (WiFi.status() == WL_CONNECTED) {
    int rssi = WiFi.RSSI();
    html += String(rssi) + " dBm";
  } else {
    html += "N/A";
  }
  
  html += R"=====(</span>
        </div>
      </div>
      
      <div class="section">
        <div class="section-title">
          <span>附近WiFi网络(扫描wifi会强制暂停所有其他操作，请谨慎操作)</span>
          <button class="refresh-btn" id="refreshBtn">
            <i class="fas fa-sync"></i> 刷新列表
          </button>
        </div>
        
        <div class="loading" id="loading">
          <div class="loading-spinner"></div>
          <p>正在扫描WiFi网络...</p>
        </div>
        
        <ul class="wifi-list" id="wifiList"></ul>
      </div>
      
      <div class="section">
        <h2 class="section-title">连接新网络</h2>
        <form id="wifiForm" onsubmit="return submitForm(event)">
          <div class="form-group">
            <label for="ssid">WiFi名称 (SSID):</label>
            <input type="text" id="ssid" name="ssid" required placeholder="可以直接在此输入wifi名称">
          </div>
          
          <div class="form-group">
            <label for="password">WiFi密码:</label>
            <input type="password" id="password" name="password" placeholder="输入WiFi密码 (开放网络请留空)">
          </div>
          
          <button type="submit" class="btn-submit">保存并连接</button>
          
          <div id="status" class="status"></div>
        </form>
      </div>
    </div>
  </div>
  <script>
    // 页面加载完成后自动加载网络列表
    window.onload = function() {
      // 隐藏加载状态
      document.getElementById('loading').style.display = 'none';
      loadNetworks();
      
      // 设置刷新按钮事件
      document.getElementById('refreshBtn').addEventListener('click', function() {
        document.getElementById('wifiList').innerHTML = '';
        document.getElementById('loading').style.display = 'block';
        
        // 发送扫描请求
        fetch('/trigger-scan')
          .then(response => response.text())
          .then(() => {
            // 等待2秒后获取结果
            setTimeout(loadNetworks, 2000);
          })
          .catch(error => {
            console.error('触发扫描错误:', error);
            document.getElementById('loading').style.display = 'none';
          });
      });
    };
    
    // 加载网络列表
    function loadNetworks() {
      const wifiList = document.getElementById('wifiList');
      const loading = document.getElementById('loading');
      
      // 显示加载状态
      loading.style.display = 'block';
      
      // 发送扫描请求
      fetch('/scan-wifi')
        .then(response => response.json())
        .then(data => {
          loading.style.display = 'none';
          wifiList.innerHTML = '';
          
          if (data.length === 0) {
            wifiList.innerHTML = '<li class="wifi-item">未找到WiFi网络</li>';
            return;
          }
          
          // 显示扫描结果
          data.forEach(network => {
            const li = document.createElement('li');
            li.className = 'wifi-item';
            li.dataset.ssid = network.ssid;
            li.dataset.strength = network.strength;
            
            // 点击效果
            li.onclick = () => {
              // 清除之前选中的效果
              document.querySelectorAll('.wifi-item').forEach(item => {
                item.style.backgroundColor = '';
              });
              
              // 设置当前选中效果
              li.style.backgroundColor = '#e3f2fd';
              
              // 填充SSID
              document.getElementById('ssid').value = network.ssid;
              
              // 聚焦到密码输入框
              document.getElementById('password').focus();
            };
            
            // 创建信号强度图标
            const signalIcons = document.createElement('div');
            signalIcons.className = 'signal-icons';
            
            // 根据信号强度创建4个小方块
            for (let i = 1; i <= 4; i++) {
              const bar = document.createElement('div');
              bar.className = 'signal-bar';
              if (i <= network.strength) bar.classList.add('active');
              signalIcons.appendChild(bar);
            }
            
            // 添加信号强度文本
            const strengthText = document.createElement('span');
            strengthText.style.marginLeft = '10px';
            strengthText.style.fontSize = '0.9rem';
            strengthText.style.color = '#666';
            
            switch(network.strength) {
              case 1: strengthText.textContent = '弱'; break;
              case 2: strengthText.textContent = '中'; break;
              case 3: strengthText.textContent = '强'; break;
              case 4: strengthText.textContent = '极强'; break;
              default: strengthText.textContent = '未知';
            }
            
            li.appendChild(signalIcons);
            li.appendChild(document.createTextNode(network.ssid));
            li.appendChild(strengthText);
            wifiList.appendChild(li);
          });
        })
        .catch(error => {
          console.error('扫描错误:', error);
          loading.style.display = 'none';
          wifiList.innerHTML = '<li class="wifi-item">扫描网络时出错，请重试</li>';
        });
    }
    
    // 表单提交处理
    function submitForm(event) {
      event.preventDefault();
      
      const ssid = document.getElementById('ssid').value;
      const password = document.getElementById('password').value;
      const statusDiv = document.getElementById('status');
      
      statusDiv.style.display = 'block';
      statusDiv.className = 'status';
      statusDiv.textContent = '正在保存配置...';
      
      // 输入验证
      if (ssid.length < 1 || ssid.length > 60) {
        statusDiv.textContent = '错误：WiFi名称长度应在1-60个字符之间';
        statusDiv.classList.add('error');
        return false;
      }
      
      // 发送数据到服务器
      fetch('/save-sta-config', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: 'ssid=' + encodeURIComponent(ssid) + '&password=' + encodeURIComponent(password)
      })
      .then(response => {
        if (!response.ok) {
          throw new Error('服务器响应错误');
        }
        return response.text();
      })
      .then(data => {
        statusDiv.textContent = '配置已保存！设备正在尝试连接新网络...';
        statusDiv.classList.add('success');
        
        // 显示倒计时
        let count = 10;
        const countdown = setInterval(() => {
          statusDiv.textContent = '配置已保存！设备将在' + count + '秒后重启...';
          count--;
          
          if (count < 0) {
            clearInterval(countdown);
            statusDiv.textContent = '设备正在重启，稍后请用同局域网设备访问屏幕上的IP';
          }
        }, 1000);
      })
      .catch(error => {
        console.error('保存错误:', error);
        statusDiv.textContent = '保存配置时出错: ' + error.message;
        statusDiv.classList.add('error');
      });
      
      return false;
    }
  </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
}
// === 处理扫描请求 ===
void handleScanWifi() {
  server.send(200, "application/json", staScanResult);
}
// === 处理触发扫描请求 ===
void handleTriggerScan() {
  staScanRequested = true;
  server.send(200, "text/plain", "Scan triggered");
}
// === 保存STA配置处理函数 ===
void handleSaveSTAConfig() {
  // 获取表单数据
  String newSSID = server.arg("ssid");
  String newPassword = server.arg("password");
  
  // 验证输入
  if (newSSID.length() == 0) {
    server.send(400, "text/plain", "SSID不能为空");
    return;
  }
  
  // 保存到Preferences
  preferences.putString("wifi_ssid", newSSID);
  preferences.putString("wifi_password", newPassword);
  
  // 返回响应
  String message = "WiFi配置已保存！设备将在10秒后重启以连接新网络";
  server.send(200, "text/plain", message);
  
  // 重启设备
  delay(10000);
  ESP.restart();
}

// === 固件更新页面 ===
void handleFirmwareUpdatePage() {
  // 从Preferences中获取当前固件版本
  String currentVersion = preferences.getString("fw_version", "未知");
  
  String html = R"=====(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>固件更新</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    
    body {
      background: linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%);
      min-height: 100vh;
      padding: 20px;
      color: #2c3e50;
    }
    
    .container {
      max-width: 600px;
      margin: 0 auto;
      background: white;
      border-radius: 15px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.1);
      overflow: hidden;
      position: relative;
    }
    
    .back-btn {
      position: absolute;
      top: 15px;
      left: 15px;
      background: #3498db;
      color: white;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.2rem;
      text-decoration: none;
      z-index: 10;
      transition: all 0.3s;
    }
    
    .back-btn:hover {
      background: #2980b9;
      transform: translateY(-2px);
      box-shadow: 0 3px 8px rgba(0,0,0,0.1);
    }
    
    header {
      background: #3498db;
      color: white;
      padding: 55px 30px 25px;
      text-align: center;
      position: relative;
    }
    
    h1 {
      font-size: 2.0rem;
      margin-bottom: 5px;
      font-weight: 600;
    }
    
    .content {
      padding: 25px;
    }
    
    .info-box {
      background: #e3f2fd;
      padding: 15px;
      border-radius: 8px;
      margin-bottom: 20px;
      text-align: center;
    }
    
    .info-label {
      font-weight: 600;
      margin-bottom: 5px;
    }
    
    .info-value {
      font-size: 1.2rem;
    }
    
    .upload-container {
      border: 2px dashed #3498db;
      border-radius: 10px;
      padding: 25px;
      text-align: center;
      margin-bottom: 20px;
      transition: all 0.3s;
      position: relative;
    }
    
    .upload-container:hover, .upload-container.drag-over {
      background-color: #f0f7ff;
      border-color: #2ecc71;
    }
    
    .upload-icon {
      font-size: 3rem;
      color: #3498db;
      margin-bottom: 15px;
    }
    
    .file-input {
      display: none;
    }
    
    .browse-btn {
      display: inline-block;
      padding: 12px 25px;
      background: #3498db;
      color: white;
      border-radius: 6px;
      cursor: pointer;
      transition: all 0.3s;
      margin-top: 15px;
      font-weight: 500;
    }
    
    .browse-btn:hover {
      background: #2980b9;
      transform: translateY(-2px);
    }
    
    #fileName {
      margin-top: 15px;
      color: #6c757d;
      font-weight: 500;
    }
    
    .btn-update {
      display: block;
      width: 100%;
      padding: 16px;
      background: #2ecc71;
      color: white;
      font-size: 1.2rem;
      font-weight: 600;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.3s;
      margin-top: 15px;
    }
    
    .btn-update:hover {
      background: #27ae60;
      transform: translateY(-3px);
      box-shadow: 0 5px 15px rgba(0,0,0,0.1);
    }
    
    .btn-update:disabled {
      background: #95a5a6;
      cursor: not-allowed;
      transform: none;
    }
    
    .status {
      margin-top: 20px;
      padding: 15px;
      border-radius: 8px;
      text-align: center;
      display: none;
    }
    
    .success {
      background: #d4edda;
      color: #155724;
      border: 1px solid #c3e6cb;
    }
    
    .error {
      background: #f8d7da;
      color: #721c24;
      border: 1px solid #f5c6cb;
    }
    
    .progress-container {
      margin-top: 20px;
      height: 20px;
      background: #f1f1f1;
      border-radius: 10px;
      overflow: hidden;
      display: none;
    }
    
    .progress-bar {
      height: 100%;
      background: #3498db;
      width: 0%;
      transition: width 0.3s;
    }
    
    #confirmModal {
      display: none;
      position: fixed;
      z-index: 1000;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0,0,0,0.6);
    }
    
    .modal-content {
      background-color: #fff;
      margin: 15% auto;
      padding: 25px;
      border-radius: 12px;
      box-shadow: 0 5px 25px rgba(0,0,0,0.3);
      width: 85%;
      max-width: 500px;
      text-align: center;
    }
    
    .modal-title {
      font-size: 1.8rem;
      margin-bottom: 20px;
      color: #e74c3c;
    }
    
    .modal-message {
      font-size: 1.2rem;
      margin-bottom: 25px;
      color: #555;
    }
    
    .modal-buttons {
      display: flex;
      justify-content: center;
      gap: 20px;
    }
    
    .modal-btn {
      padding: 12px 30px;
      border: none;
      border-radius: 6px;
      font-size: 1.1rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.3s;
      min-width: 120px;
    }
    
    .modal-confirm {
      background: #e74c3c;
      color: white;
    }
    
    .modal-confirm:hover {
      background: #c0392b;
    }
    
    .modal-cancel {
      background: #95a5a6;
      color: white;
    }
    
    .modal-cancel:hover {
      background: #7f8c8d;
    }

  </style>
</head>
<body>
  <div class="container">
    <a href="/" class="back-btn">
      ←
    </a>
    
    <header>
      <h1>固件更新</h1>
    </header>
    
    <div class="content">
      <div class="info-box">
        <div class="info-label">当前固件版本</div>
        <div class="info-value" id="currentVersion">)=====";
  
  html += currentVersion;
  
  html += R"=====(</div>
      </div>
      
      <div class="upload-container" id="uploadContainer">
        <div class="upload-icon">📁</div>
        <h3>拖放固件文件到此处</h3>
        <p>或点击下方按钮选择文件</p>
        <p>仅支持.bin格式的固件文件</p>
        <p>固件更新不会清除保存的所有配置，如需清除请恢复出厂设置</p>
        <input type="file" id="firmwareFile" class="file-input" accept=".bin">
        <label for="firmwareFile" class="browse-btn">选择固件文件</label>
        <div id="fileName">未选择文件</div>
      </div>
      
      <button class="btn-update" id="updateBtn" disabled>开始更新</button>
      
      <div class="progress-container" id="progressContainer">
        <div class="progress-bar" id="progressBar"></div>
      </div>
      
      <div id="status" class="status"></div>
    </div>
  </div>
  
  <div id="confirmModal" class="modal">
    <div class="modal-content">
      <h2 class="modal-title">确认固件更新</h2>
      <p class="modal-message">您确定要更新固件吗？</p>
      <p class="modal-message">更新过程中请勿断电，否则可能导致设备损坏。</p>
      <p class="modal-message">文件名: <span id="confirmFileName"></span></p>
      
      <div class="modal-buttons">
        <button class="modal-btn modal-cancel" id="cancelUpdate">取消</button>
        <button class="modal-btn modal-confirm" id="confirmUpdate">确认更新</button>
      </div>
    </div>
  </div>
  
  <script>
    const firmwareFile = document.getElementById('firmwareFile');
    const fileName = document.getElementById('fileName');
    const updateBtn = document.getElementById('updateBtn');
    const progressContainer = document.getElementById('progressContainer');
    const progressBar = document.getElementById('progressBar');
    const statusDiv = document.getElementById('status');
    const confirmModal = document.getElementById('confirmModal');
    const confirmFileName = document.getElementById('confirmFileName');
    const cancelUpdateBtn = document.getElementById('cancelUpdate');
    const confirmUpdateBtn = document.getElementById('confirmUpdate');
    const uploadContainer = document.getElementById('uploadContainer');
    
    let selectedFile = null;
    
    // 文件选择处理
    firmwareFile.addEventListener('change', function(e) {
      handleFileSelect(this.files);
    });
    
    // 拖放处理
    uploadContainer.addEventListener('dragover', function(e) {
      e.preventDefault();
      e.stopPropagation();
      this.classList.add('drag-over');
    });
    
    uploadContainer.addEventListener('dragleave', function(e) {
      e.preventDefault();
      e.stopPropagation();
      this.classList.remove('drag-over');
    });
    
    uploadContainer.addEventListener('drop', function(e) {
      e.preventDefault();
      e.stopPropagation();
      this.classList.remove('drag-over');
      
      if (e.dataTransfer.files && e.dataTransfer.files.length > 0) {
        handleFileSelect(e.dataTransfer.files);
      }
    });
    
    // 处理选择的文件（支持拖放和文件选择）
    function handleFileSelect(files) {
      if (files.length > 0) {
        selectedFile = files[0];
        fileName.textContent = selectedFile.name;
        
        // 检查文件扩展名
        if (!selectedFile.name.toLowerCase().endsWith('.bin')) {
          statusDiv.textContent = '错误：请选择.bin格式的固件文件';
          statusDiv.className = 'status error';
          statusDiv.style.display = 'block';
          updateBtn.disabled = true;
          return;
        }
        
        // 启用更新按钮
        updateBtn.disabled = false;
        
        // 清除之前的错误信息
        statusDiv.style.display = 'none';
        
      } else {
        selectedFile = null;
        fileName.textContent = '未选择文件';
        updateBtn.disabled = true;
      }
    }
    
    // 更新按钮点击
    updateBtn.addEventListener('click', function() {
      if (!selectedFile) return;
      
      // 在确认框中显示文件名
      confirmFileName.textContent = selectedFile.name;
      confirmModal.style.display = 'block';
    });
    
    // 模态框按钮事件
    cancelUpdateBtn.addEventListener('click', function() {
      confirmModal.style.display = 'none';
    });
    
    confirmUpdateBtn.addEventListener('click', function() {
      confirmModal.style.display = 'none';
      startUpdate();
    });
    
    // 开始固件更新
    function startUpdate() {
      if (!selectedFile) return;
      
      const formData = new FormData();
      formData.append('firmware', selectedFile);
      
      // 显示进度条
      progressContainer.style.display = 'block';
      progressBar.style.width = '0%';
      statusDiv.style.display = 'none';
      updateBtn.disabled = true;
      
      // 创建XMLHttpRequest对象以支持进度显示
      const xhr = new XMLHttpRequest();
      
      // 上传进度处理
      xhr.upload.addEventListener('progress', function(e) {
        if (e.lengthComputable) {
          const percent = Math.round((e.loaded / e.total) * 100);
          progressBar.style.width = percent + '%';
        }
      });
      
      // 上传完成处理
      xhr.addEventListener('load', function() {
        if (xhr.status === 200) {
          statusDiv.textContent = '固件更新成功！设备将重启';
          statusDiv.className = 'status success';
          statusDiv.style.display = 'block';
          
          // 倒计时重启
          setTimeout(() => {
            window.location.href = '/';
          }, 3000);
        } else {
          statusDiv.textContent = '更新失败: ' + xhr.responseText;
          statusDiv.className = 'status error';
          statusDiv.style.display = 'block';
          progressContainer.style.display = 'none';
          updateBtn.disabled = false;
        }
      });
      
      // 错误处理
      xhr.addEventListener('error', function() {
        statusDiv.textContent = '网络错误，请重试';
        statusDiv.className = 'status error';
        statusDiv.style.display = 'block';
        progressContainer.style.display = 'none';
        updateBtn.disabled = false;
      });
      
      // 发送请求
      xhr.open('POST', '/update-firmware');
      xhr.send(formData);
    }
  </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
}

// === 固件上传处理函数 ===
void handleFirmwareUpdate() {
  HTTPUpload& upload = server.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    if (updateInProgress) {
      upload.status = UPLOAD_FILE_ABORTED;
      return;
    }
    
    updateSize = 0;
    
    // 开始固件更新
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
      upload.status = UPLOAD_FILE_ABORTED;
      return;
    }
    updateInProgress = true;
    
  } else if (upload.status == UPLOAD_FILE_WRITE && updateInProgress) {
    // 写入固件数据
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      upload.status = UPLOAD_FILE_ABORTED;
      return;
    }
    updateSize += upload.currentSize;
    
  } else if (upload.status == UPLOAD_FILE_END && updateInProgress) {
    if (Update.end(true)) {
      server.send(200, "text/plain", "更新成功! 设备将重启");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "更新失败");
    }
    updateInProgress = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED && updateInProgress) {
    Update.end();
    updateInProgress = false;
    Serial.println("固件更新已中止");
  }
}

// === WIFI重连 ===
bool reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED || apMode) return true;
  
  Serial.println("WiFi disconnected! Attempting to reconnect...");
  
  if (ssid == "") {
    Serial.println("No saved WiFi credentials");
    return false;
  }

  // 确保WiFi模式正确
  WiFi.mode(WIFI_STA);
  
  // 尝试连接
  WiFi.disconnect(true); // 清理之前的连接
  delay(500); // 短暂延迟确保断开
  
  if (password == NULL) {
    WiFi.begin(ssid); // 开放式网络
  } else {
    WiFi.begin(ssid, password); // 加密网络
  }
  
  // 等待连接建立，最多尝试15秒
  unsigned long startTime = millis();
  int attempts = 0;
  const int MAX_ATTEMPTS = 15;
  
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_ATTEMPTS) {
    delay(1000);
    attempts++;
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi reconnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nWiFi reconnect failed after multiple attempts");
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    return false;
  }
}

void handleTempUpdate() {
  // 创建JSON响应
  String json = "{";
  json += "\"temp\":" + String(currentTemp, 1) + ",";
  if (chamberSensorConnected) {
    json += "\"chamberTemp\":" + String(chamberTemp, 1) + ",";
  } else {
    json += "\"chamberTemp\":null,";
  }
  if (aht30Connected) {
    json += "\"aht30Temperature\":" + String(aht30Temperature, 1) + ",";
    json += "\"aht30Humidity\":" + String(aht30Humidity, 1) + ",";
  } else {
    json += "\"aht30Temperature\":null,";
    json += "\"aht30Humidity\":null,";
  }

  // 基准时间
  unsigned long currentMillis = millis();
  json += "\"baseTime\":" + String(currentMillis) + ",";

  // 历史数据
  json += "\"history\":[";
  bool first = true;
  unsigned long currentTime = millis();
  
  // 返回最近的数据
  for (int i = 0; i < HISTORY_SIZE_web; i++) {
    int idx = (historyIndex_web - 1 - i + HISTORY_SIZE_web) % HISTORY_SIZE_web;
    
    // 只返回最近5分钟内的数据
    if (currentTime - tempHistory_web[idx].timestamp_web <= 300000) {
      if (!first) {
        json += ",";
      }
      json += "{";
      json += "\"time\":" + String(tempHistory_web[idx].timestamp_web) + ",";
      json += "\"mainTemp\":" + String(tempHistory_web[idx].mainTemp_web, 1) + ",";
      if (!isnan(tempHistory_web[idx].chamberTemp_web)) {
        json += "\"chamberTemp\":" + String(tempHistory_web[idx].chamberTemp_web, 1);
      } else {
        json += "\"chamberTemp\":0";
      }
      json += "}";
      first = false;
    }
  }
  json += "]";
  
  json += "}";
  
  server.send(200, "application/json", json);
}

// === 按键处理函数 ===
void handleButtons() {
  unsigned long now = millis();

  // 读取按键状态（低电平为按下）
  bool tempPressed = digitalRead(TEMP_BUTTON_PIN) == LOW;
  bool timePressed = digitalRead(TIME_BUTTON_PIN) == LOW;
  bool modePressed = digitalRead(MODE_BUTTON_PIN) == LOW;
  bool factoryPressed = digitalRead(FACTORY_BUTTON_PIN) == LOW;
  bool tempTimeCombo = tempPressed && timePressed;

  // 同时长按温度+时间 -> PID整定
  if (tempTimeCombo) {
    if (!pidHoldActive) {
      PIDPressStart = now;
      pidHoldActive = true;
    } else if (now - PIDPressStart >= LONG_PRESS_DURATION &&
               currentState == STANDBY && autotuneState == ATUNE_OFF) {
      startAutoTune();
      Serial.println("Start PID autotune");
      pidHoldActive = false;
      PIDPressStart = now;
    }
  } else {
    pidHoldActive = false;
  }

  // 温度按钮：单击+5，长按-5
  if (tempPressed != lastTempButtonState) {
    if (tempPressed) {
      tempButtonPressStart = now;
    } else {
      unsigned long pressDuration = now - tempButtonPressStart;
      if (!tempTimeCombo && !tempButtonLongPressed &&
          pressDuration >= DEBOUNCE_DELAY && pressDuration < LONG_PRESS_DURATION &&
          currentState == STANDBY && autotuneState == ATUNE_OFF) {
        targetTemp += 5;
        if (targetTemp > MAX_TEMP) targetTemp = MAX_TEMP;
        currentPreset = 0; // 切换到自定义
        preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
        preferences.putDouble("targetTemp", targetTemp);
        publishState();
        Serial.println("Temp +5");
      }
      tempButtonLongPressed = false;
    }
    lastTempButtonState = tempPressed;
  }

  if (tempPressed && !tempTimeCombo && currentState == STANDBY && autotuneState == ATUNE_OFF) {
    unsigned long held = now - tempButtonPressStart;
    if (!tempButtonLongPressed && held >= LONG_PRESS_DURATION) {
      tempButtonLongPressed = true;
      targetTemp -= 5;
      if (targetTemp < MIN_TEMP) targetTemp = MIN_TEMP;
      currentPreset = 0; // 切换到自定义
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putDouble("targetTemp", targetTemp);
      publishState();
      Serial.println("Temp -5");
      tempButtonPressStart = now;
    } else if (tempButtonLongPressed && held >= LONG_PRESS_REPEAT) {
      targetTemp -= 5;
      if (targetTemp < MIN_TEMP) targetTemp = MIN_TEMP;
      currentPreset = 0; // 切换到自定义
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putDouble("targetTemp", targetTemp);
      publishState();
      Serial.println("Temp -5");
      tempButtonPressStart = now;
    }
  }

  // 时间按钮：单击+30，长按-30
  if (timePressed != lastTimeButtonState) {
    if (timePressed) {
      timeButtonPressStart = now;
    } else {
      unsigned long pressDuration = now - timeButtonPressStart;
      if (!tempTimeCombo && !timeButtonLongPressed &&
          pressDuration >= DEBOUNCE_DELAY && pressDuration < LONG_PRESS_DURATION &&
          currentState == STANDBY && autotuneState == ATUNE_OFF) {
        targetTime += 30;
        if (targetTime > 1440) targetTime = 1440;
        currentPreset = 0; // 切换到自定义
        preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
        preferences.putInt("targetTime", targetTime);
        publishState();
        Serial.println("Time +30min");
      }
      timeButtonLongPressed = false;
    }
    lastTimeButtonState = timePressed;
  }

  if (timePressed && !tempTimeCombo && currentState == STANDBY && autotuneState == ATUNE_OFF) {
    unsigned long held = now - timeButtonPressStart;
    if (!timeButtonLongPressed && held >= LONG_PRESS_DURATION) {
      timeButtonLongPressed = true;
      targetTime -= 30;
      if (targetTime < 0) targetTime = 0;
      currentPreset = 0; // 切换到自定义
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putInt("targetTime", targetTime);
      publishState();
      Serial.println("Time -30min");
      timeButtonPressStart = now;
    } else if (timeButtonLongPressed && held >= LONG_PRESS_REPEAT) {
      targetTime -= 30;
      if (targetTime < 0) targetTime = 0;
      currentPreset = 0; // 切换到自定义
      preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
      preferences.putInt("targetTime", targetTime);
      publishState();
      Serial.println("Time -30min");
      timeButtonPressStart = now;
    }
  }

  // 模式按钮：单击切换预设；长按开始/取消烘干或取消PID整定
  if (modePressed != lastModeButtonState) {
    if (modePressed) {
      modeButtonPressStart = now;
    } else {
      unsigned long pressDuration = now - modeButtonPressStart;
      if (!modeButtonLongPressed &&
          pressDuration >= DEBOUNCE_DELAY && pressDuration < LONG_PRESS_DURATION &&
          currentState == STANDBY && autotuneState == ATUNE_OFF) {
        currentPreset = (currentPreset + 1) % (PRESET_COUNT + 1);
        if (currentPreset == 0) currentPreset = 1;
        if (currentPreset > 0) {
          targetTemp = (double)PRESETS[currentPreset-1].temp;
          targetTime = PRESETS[currentPreset-1].time;
        }
        preferences.putInt(PREF_KEY_CURRENT_PRESET, currentPreset);
        publishState();
        Serial.println("Mode changed");
      }
      modeButtonLongPressed = false;
      isMOdeButtonLongPressed = false;
    }
    lastModeButtonState = modePressed;
  }

  if (modePressed && !isMOdeButtonLongPressed &&
      now - modeButtonPressStart >= LONG_PRESS_DURATION) {
    modeButtonLongPressed = true;
    isMOdeButtonLongPressed = true;

    if (currentState == STANDBY && autotuneState == ATUNE_OFF) {
      startDrying();
      Serial.println("Start drying");
    } else if (currentState == DRYING) {
      stopDrying(true);
      Serial.println("Stop drying");
    } else if (autotuneState != ATUNE_OFF) {
      finishAutoTune(false); // 按键取消整定，统一使用收尾逻辑恢复PID
      Serial.println("Cancel autotune");
    }
    modeButtonPressStart = now;
  }

  // 恢复出厂设置按钮
  if (factoryPressed != lastFactoryButtonState) {
    if (factoryPressed) {
      factoryButtonPressStart = now;
      factoryButtonLongPressed = false;
    } else {
      // 保留为单击扩展
    }
    lastFactoryButtonState = factoryPressed;
  }

  if (factoryPressed && 
      now - factoryButtonPressStart >= FACTORY_DELAY &&
      !factoryButtonLongPressed) {
    display.clearDisplay();
    drawZh(4,8,factory_reset_120x20,display);
    drawZh(4,28,clear_all_config_120x20,display);
    drawZh(39,52,six_points_50x8,display);
    display.display();
    Serial.println("Restore Factory Settings");
    delay(2000);
    // 清除所有配置
    preferences.clear();
    preferences.end();
    delay(500);
    ESP.restart();
  }
}

void controlExhaustValve(bool open) {
  if (exhaustActionInProgress) return; // 如果已有动作在进行中，则忽略新请求
  
  exhaustActionInProgress = true;
  exhaustActionStart = millis();
  
  if (open) {
    exhaustActionType = "open";
    if (exhaustMotorReversed) {
      // 反转方向：A低B高为打开
      digitalWrite(EXHAUST_INA_PIN, LOW);
      digitalWrite(EXHAUST_INB_PIN, HIGH);
    } else {
      // 正常方向：A高B低为打开
      digitalWrite(EXHAUST_INA_PIN, HIGH);
      digitalWrite(EXHAUST_INB_PIN, LOW);
    }
  } else {
    exhaustActionType = "close";
    if (exhaustMotorReversed) {
      // 反转方向：A高B低为关闭
      digitalWrite(EXHAUST_INA_PIN, HIGH);
      digitalWrite(EXHAUST_INB_PIN, LOW);
    } else {
      // 正常方向：A低B高为关闭
      digitalWrite(EXHAUST_INA_PIN, LOW);
      digitalWrite(EXHAUST_INB_PIN, HIGH);
    }
  }
}

void stopExhaustMotor() {
  digitalWrite(EXHAUST_INA_PIN, LOW);
  digitalWrite(EXHAUST_INB_PIN, LOW);
}

void updateExhaustValve() {
  if (exhaustActionInProgress) {
    unsigned long actionDuration = exhaustActionTime;
    if (exhaustActionType == "close") {
      actionDuration += EXHAUST_CLOSE_EXTRA_MS;
    }

    if (millis() - exhaustActionStart >= actionDuration) {
      stopExhaustMotor();
      exhaustActionInProgress = false;
      
      if (exhaustActionType == "open") {
        exhaustValveOpen = true;
      } else if (exhaustActionType == "close") {
        exhaustValveOpen = false;
      }
      
      exhaustActionType = "";
    }
  }
}

void handleExhaustCycle() {
  if (currentState == DRYING && exhaustCycleActive) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastExhaustCycleTime;
    
    if (exhaustCycleState) {
      // 当前处于开启状态，检查是否到达关闭时间
      if (elapsedTime >= exhaustOnDuration) {
        closeExhaustValve();
        exhaustCycleState = false;
        lastExhaustCycleTime = currentTime;
      }
    } else {
      // 当前处于关闭状态，检查是否到达开启时间
      if (elapsedTime >= exhaustOffDuration) {
        openExhaustValve();
        exhaustCycleState = true;
        lastExhaustCycleTime = currentTime;
      }
    }
  }
}

void setExhaustActionTime(unsigned long timeMs) {
  if (timeMs >= 3000 && timeMs <= 20000) {
    exhaustActionTime = timeMs;
    preferences.putULong(PREF_KEY_EXH_ACTION_TIME, exhaustActionTime);
  }
}

bool setExhaustFirstDelayMinutes(unsigned long minutes) {
  if (minutes <= 300) {
    exhaustFirstDelay = minutes * 60000UL;
    preferences.putULong(PREF_KEY_EXH_FIRST_DELAY, exhaustFirstDelay);
    return true;
  }
  return false;
}

bool setExhaustOnDurationMinutes(unsigned long minutes) {
  if (minutes >= 1 && minutes <= 300) {
    exhaustOnDuration = minutes * 60000UL;
    preferences.putULong(PREF_KEY_EXH_ON_DURATION, exhaustOnDuration);
    return true;
  }
  return false;
}

bool setExhaustOffDurationMinutes(unsigned long minutes) {
  if (minutes >= 1 && minutes <= 300) {
    exhaustOffDuration = minutes * 60000UL;
    preferences.putULong(PREF_KEY_EXH_OFF_DURATION, exhaustOffDuration);
    return true;
  }
  return false;
}

void toggleExhaustMotorDirection() {
  exhaustMotorReversed = !exhaustMotorReversed;
  preferences.putBool(PREF_KEY_EXH_MOTOR_REV, exhaustMotorReversed);
}

void openExhaustValve() {
  if (!exhaustActionInProgress) {
    controlExhaustValve(true);
  }
}

void closeExhaustValve() {
  if (!exhaustActionInProgress) {
    controlExhaustValve(false);
  }
}

String getExhaustStateCode() {
  if (exhaustActionInProgress) {
    if (exhaustActionType == "open") return "opening";
    if (exhaustActionType == "close") return "closing";
    return "moving";
  }
  return exhaustValveOpen ? "open" : "closed";
}

String getExhaustStateLabel() {
  String code = getExhaustStateCode();
  if (code == "opening") return "开启中";
  if (code == "closing") return "关闭中";
  if (code == "open") return "打开";
  return "关闭";
}

// === 湿度自动烘干 ===
bool setAutoDryEnabled(bool enabled) {
  autoDryEnabled = enabled;
  preferences.putBool("autoDryEnabled", autoDryEnabled);
  return true;
}

static bool validateHumidityRange(float humidity) {
  return humidity >= 0.0f && humidity <= 100.0f && isfinite(humidity);
}

bool setAutoDryStart(float humidity) {
  if (!validateHumidityRange(humidity)) return false;
  autoDryStartHumidity = humidity;
  preferences.putFloat("autoDryStartH", autoDryStartHumidity);
  return true;
}

bool setAutoDryRunMinutes(int minutes) {
  if (minutes <= 0) return false;
  if (minutes > 1440) minutes = 1440;
  autoDryRunMinutes = minutes;
  preferences.putInt("autoDryRunMin", autoDryRunMinutes);
  return true;
}

bool setAutoDryCooldownMinutes(int minutes) {
  if (minutes < 0) return false;
  if (minutes > 1440) minutes = 1440;
  autoDryCooldownMinutes = minutes;
  preferences.putInt(PREF_KEY_AUTO_DRY_COOLDOWN, autoDryCooldownMinutes);
  return true;
}

bool setPidAutotuneTarget(float target) {
  if (!isfinite(target)) return false;
  if (target < MIN_TEMP) target = MIN_TEMP;
  if (target > MAX_TEMP) target = MAX_TEMP;
  pidAutotuneTarget = target;
  preferences.putDouble("pidAutoTarget", pidAutotuneTarget);
  return true;
}

void handleAutoDry() {
  if (!autoDryEnabled) return;
  if (!aht30Connected) return;
  if (!isfinite(aht30Humidity)) return;
  if (autotuneState != ATUNE_OFF) return;

  // 自动烘干状态切换：运行期间不重复触发，结束后再检测
  if (autoDryRunning && currentState == STANDBY) {
    autoDryRunning = false; // 一轮结束，允许下一轮触发
  }

  // 用户手动停止后的一段时间内不触发自动烘干
  if (autoDryBlockUntil != 0) {
    long diff = (long)(millis() - autoDryBlockUntil);
    if (diff < 0) {
      return;
    } else {
      autoDryBlockUntil = 0;
    }
  }

  if (!autoDryRunning && currentState == STANDBY) {
    if (aht30Humidity >= autoDryStartHumidity) {
      autoDryRunning = true;
      autoDryPrevTargetTime = targetTime;
      int runMin = autoDryRunMinutes;
      if (runMin <= 0) runMin = 1;
      targetTime = runMin;
      startDrying();
      // 恢复用户设定时间，计时已由remainingTime接管
      targetTime = autoDryPrevTargetTime;
    }
  }
}