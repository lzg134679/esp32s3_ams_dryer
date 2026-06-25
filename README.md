# AMS Lite 智能烘干器

基于 ESP32-S3 的 3D 打印耗材智能烘干器固件，支持精确温度控制、自动除湿、远程监控等功能。
- [B站视频链接](https://www.bilibili.com/video/BV1hrrPBKE2C)
- [嘉立创开源硬件平台](https://oshwhub.com/soughtquinee/ams-lite-dryer)
- [makerworld中国区模型文件](https://makerworld.com.cn/zh/models/1992331-gua-pei-ams-litefeng-xiang-de-zhi-neng-hong-gan-qi#profileId-2218450)
- [makerworld国际区模型文件](https://makerworld.com/zh/models/2220547-smart-dryer-for-ams-lite-enclosure#profileId-2414835)

# 烧录说明
- 老版本都反馈烧录后一直频繁重启，1.0.5版本修复了这个问题。
- 需要同时烧录`bootloader.bin`、`partitions.bin`、`烘干器V1.0.5.bin`三个文件
- 注意烧录位置分别为 `0x0000`、`0x8000`、`0x10000`
- 烧录软件在release里的1.0.5版本压缩包内，默认已经配置好ESP32-S3的烧录参数，直接选择串口开始烧录即可
![烧录软件详情](https://s3.bmp.ovh/2026/06/25/MH3etcHE.png)

# 功能特性

## 功能特性

### 核心功能

- **PID温度控制**：支持自动整定功能，精准控温
- **材料预设**：内置 PLA/PETG/ABS/ASA/PA 五种材料预设
- **定时烘干**：支持0-1440分钟定时设置
- **湿度自动烘干**：根据环境湿度自动启动/停止烘干
- **排气阀控制**：支持自动/手动模式，可配置开关周期

### 安全保护

- **超温保护**：温度超过设定值自动触发保护
- **传感器异常检测**：自动检测传感器连接状态
- **看门狗复位记录**：记录设备重启原因

### 网络功能

- **WiFi配网**：支持STA模式连接和AP模式配网
- **MQTT通信**：支持 Home Assistant 集成
- **Web界面**：本地网页控制界面
- **固件升级**：支持远程OTA升级

### 显示与交互

- **OLED显示屏**：128x64 分辨率，中文界面
- **物理按键**：温度/时间/模式/重置四个按键
- **LED指示灯**：状态指示

## 硬件配置

### 引脚定义

| 功能        | 引脚     | 说明          |
| --------- | ------ | ----------- |
| 主温控器数据    | GPIO4  | DS18B20     |
| 仓温传感器     | GPIO16 | DS18B20（可选） |
| LED       | GPIO48 | NeoPixel    |
| OLED SDA  | GPIO6  | I2C         |
| OLED SCL  | GPIO5  | I2C         |
| AHT30 SDA | GPIO9  | I2C         |
| AHT30 SCL | GPIO10 | I2C         |
| 温度按钮      | GPIO17 | 输入（上拉）      |
| 时间按钮      | GPIO18 | 输入（上拉）      |
| 模式按钮      | GPIO8  | 输入（上拉）      |
| 重置按钮      | GPIO19 | 输入（上拉）      |
| 加热器PWM    | GPIO7  | LEDC通道0     |
| 风扇PWM     | GPIO15 | LEDC通道1     |
| 排气阀INA    | GPIO11 | 电机控制        |
| 排气阀INB    | GPIO12 | 电机控制        |

### 硬件清单

- ESP32-S3 DevKitC-1
- SSD1306 128x64 OLED显示屏
- AHT30 温湿度传感器
- DS18B20 温度传感器（2个）
- PTC加热器模块
- 风扇
- 排气阀电机

## 软件配置

### PlatformIO 环境

项目使用 PlatformIO 进行开发和编译，确保已安装以下依赖库：

- `adafruit/Adafruit SSD1306`
- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit NeoPixel`
- `adafruit/Adafruit AHTX0`
- `milesburton/DallasTemperature`
- `knolleary/PubSubClient`
- `bblanchon/ArduinoJson`

### WiFi配置

设备首次启动时，若未保存WiFi配置或连接失败，将自动进入AP模式：

- **SSID**: `ams_lite_dryer`
- **密码**: 无

连接AP后访问 `192.168.4.1` 配置WiFi网络。

### MQTT配置

设备支持通过MQTT连接Home Assistant，在Web界面配置以下参数：

- MQTT服务器地址
- MQTT端口（默认1883）
- MQTT用户名（可选）
- MQTT密码（可选）

## Home Assistant 集成

### MQTT主题

#### 状态主题（State）

| 主题                           | 说明      | 单位       |
| ---------------------------- | ------- | -------- |
| `dryer/state/temp`           | 当前温度    | °C       |
| `dryer/state/target_temp`    | 目标温度    | °C       |
| `dryer/state/chamber_temp`   | 仓温      | °C       |
| `dryer/state/aht30_temp`     | AHT30温度 | °C       |
| `dryer/state/aht30_humidity` | AHT30湿度 | %        |
| `dryer/state/time`           | 设定时间    | min      |
| `dryer/state/remaining`      | 剩余时间    | s        |
| `dryer/state/mode`           | 当前模式    | -        |
| `dryer/state/status`         | 设备状态    | -        |
| `dryer/state/pid`            | PID参数   | Kp-Ki-Kd |
| `dryer/state/ip`             | 设备IP    | -        |
| `dryer/state/fan`            | 风扇速度    | %        |
| `dryer/state/exhaust`        | 排气阀状态   | -        |
| `dryer/alerts`               | 警告信息    | -        |

#### 控制主题（Control）

| 主题                           | 说明    | 有效取值                         |
| ---------------------------- | ----- | ---------------------------- |
| `dryer/control/temp`         | 设置温度  | 40-100                       |
| `dryer/control/time`         | 设置时间  | 0-1440                       |
| `dryer/control/mode`         | 设置模式  | CUSTOM/PLA/PETG/ABS/ASA/PA   |
| `dryer/control/power`        | 开始烘干  | 任意值                          |
| `dryer/control/stop`         | 停止烘干  | 任意值                          |
| `dryer/control/autotune`     | PID整定 | START                        |
| `dryer/control/fan`          | 风扇速度  | 39-100                       |
| `dryer/control/exhaust`      | 排气阀控制 | OPEN/CLOSE/TOGGLE\_DIRECTION |
| `dryer/control/exhaust_mode` | 排气阀模式 | AUTO/MANUAL                  |

### Home Assistant YAML配置

参考 `src/HA-configuration.yaml` 文件，包含完整的传感器、选择器、开关和按钮配置。

## 材料预设

| 材料   | 温度   | 时间   |
| ---- | ---- | ---- |
| PLA  | 55°C | 6小时  |
| PETG | 65°C | 8小时  |
| ABS  | 80°C | 8小时  |
| ASA  | 80°C | 8小时  |
| PA   | 80°C | 12小时 |

## 按键操作

### 短按（< 0.5秒）

- **温度键**：增加目标温度（+5°C）
- **时间键**：增加烘干时间（+30分钟）
- **模式键**：切换材料预设
- **重置键**：无作用

### 长按（> 0.5秒）

- **温度键**：减少目标温度（-5°C）
- **时间键**：减少烘干时间（-30分钟）
- **模式键**：开始/停止烘干
- **重置键**：恢复出厂设置（需长按15秒）

## Web界面功能

设备连接WiFi后，可通过IP地址访问以下页面：

- `/` - 主控制页面
- 下方其他页面都可以通过主页跳转
- `/mqtt-config` - MQTT配置页面
- `/wifi-config` - WiFi配置页面
- `/firmware-update` - 固件升级页面
- `/api/status` - 状态API（JSON）
- `/api/temp` - 温度历史API
- `/api/performance` - 性能数据API

## 安全说明

1. **超温保护**：当温度超过设定温度10°C时自动关闭加热器
2. **风扇保护**：风扇最小速度限制，防止过热
3. **传感器异常**：传感器断开时自动停止加热
4. **看门狗**：支持软件看门狗，异常自动重启

## 开发说明

### 编译与上传

```bash
# 编译项目
pio run

# 上传固件
pio run --target upload

# 监控串口
pio device monitor -b 115200
```

### 固件版本

在 `platformio.ini` 中定义固件版本，例如：

```ini
build_flags = 
  -DFIRMWARE_VERSION="1.0.0"
```

## 故障排除

### 常见问题

| 问题       | 原因        | 解决方案       |
| -------- | --------- | ---------- |
| WiFi连接失败 | SSID或密码错误 | 进入AP模式重新配置 |
| MQTT连接失败 | 服务器配置错误   | 检查MQTT配置   |
| 温度显示异常   | 传感器连接问题   | 检查接线       |
| 加热器不工作   | 继电器或PWM问题 | 检查硬件连接     |
| 排气阀不动    | 电机方向或时间配置 | 调整排气阀参数    |

### 日志查看

通过串口监控查看设备日志，波特率115200。
