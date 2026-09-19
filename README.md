# ESP_GPS_Tracker

基于 **ESP8266 (NodeMCU v2) + GPS 模块** 的业余无线电 APRS 追踪器，同时把位置上报到 **APRS-IS** 和 **Traccar**，内置 **SmartBeacon 智能信标算法**、**Web 配置界面**与 **OTA 升级**。

> 作者：Charles Cui（业余无线电呼号 BH9FXK）
> 当前版本：v0.6

## 功能特性
- GPS 数据采集（NMEA，TinyGPSPlus 解析）
- APRS-IS 上报（TCP 14580，业余无线电自动位置报告系统）
- SmartBeacon 智能信标：根据速度与转弯自动调节上报频率，省电省带宽
- Traccar 上报（HTTP，osmand 协议，**可选**）
- Web 配置界面（LittleFS 静态页面）：APRS / Traccar / WiFi 参数均可网页填写
- 初始 AP 配网模式 + ArduinoOTA 无线升级

## 硬件清单
- NodeMCU v2（ESP-12E，ESP8266）
- GPS 模块（支持 NMEA 输出，如 u-blox 系列），波特率 9600

### 接线
| ESP8266 引脚 | GPS 模块 |
|-------------|----------|
| D6 (GPIO12) | GPS TX   |
| D7 (GPIO13) | GPS RX   |
| 3.3V        | VCC      |
| GND         | GND      |

> 注意：ESP8266 的 IO 为 3.3V 电平，请勿直接连接 5V 的 GPS 模块 TTL 输出。

## 编译与烧录
1. 安装 [PlatformIO](https://platformio.org/)。
2. 克隆仓库并用 PlatformIO 打开。
3. 烧录固件：
   ```
   pio run -t upload
   ```
   默认通过 OTA 上传到 `192.168.31.143`（见 `platformio.ini` 的 `upload_port`，按需修改）。
4. 上传文件系统（Web 配置页面与样式）：
   ```
   pio run -t uploadfs
   ```
   `platformio.ini` 中已设置 `board_build.filesystem = littlefs`。

首次上电后设备会启动一个名为 `aprs-tracker`、密码 `88888888` 的 Wi-Fi 热点，用于初始配置（见下方安全提示）。

## Web 配置流程
1. 手机/电脑连接 `aprs-tracker` 热点。
2. 浏览器打开 `http://192.168.4.1/`。
3. 依次配置：
   - **APRS config（必填）**：呼号（如 `BH9FXK-5`）、APRS passcode、注释、自定义信息、APRS-IS 服务器（如 `asia.aprs2.net`）、符号、SmartBeacon 参数。
   - **Traccar config（选填）**：设备 ID、服务器域名、端口（默认 5055）。**留空则不启用 Traccar 上报**，设备仅向 APRS-IS 发信标。
   - **WiFi config**：填写两个 SSID/密码，设备会按序自动连接。
4. 点击 **Exit & boot** 重启，进入正常追踪模式。

## SmartBeacon 参数说明
| 参数 | 含义 | 建议 |
|------|------|------|
| Low Speed | 低于此速度视为“静止” | 3 km/h |
| Low Rate | 静止时的上报间隔（秒） | 300 s |
| High Speed | 达到此速度用最快间隔 | 60 km/h |
| High Rate | 最快上报间隔（秒） | 60 s |
| Turn Minimum Angle | 触发转弯上报的最小角度 | 8° |
| Turn Slope | 速度相关的转弯灵敏度 | 255 |
| Turn Minimum Rate | 转弯上报的最小间隔（秒） | 5 s |

## 安全提示
- 初始 AP 密码为硬编码 `88888888`，**配置完成后请尽快修改**（见 `src/main.cpp` 中 `setup()` 的 `WiFi.softAP(...)`）。
- 配置文件（含 Wi-Fi 密码、APRS passcode）以明文存储在 LittleFS。
- Web 配置接口与 `/dl` 下载接口默认无认证，请勿在不可信网络长期使用。
- 位置数据通过明文通道传输（APRS-IS 14580 本就明文；Traccar 默认 HTTP）。

## 许可
本项目采用 [MIT License](./LICENSE)。
