# ESP_GPS_Tracker

基于 **ESP8266 (NodeMCU v2) + GPS 模块** 的业余无线电 APRS 追踪器，同时把位置上报到 **APRS-IS** 和 **Traccar**，内置 **SmartBeacon 智能信标算法**与 **Web 配置界面**。

> 作者：Charles Cui（业余无线电呼号 BH9FXK）
> 当前版本：v0.11

## 功能特性
- GPS 数据采集（NMEA，TinyGPSPlus 解析）
- APRS-IS 上报（TCP 14580，业余无线电自动位置报告系统）
- SmartBeacon 智能信标：根据速度与转弯自动调节上报频率，省电省带宽
- Traccar 上报（HTTP，osmand 协议，**可选**）
- Web 配置界面（LittleFS 静态页面）：APRS / Traccar / WiFi 参数均可网页填写
- 初始 AP 配网模式，参数网页填写后自动重启进入追踪模式
- 固件更新走 USB 串口（v0.7 起移除 ArduinoOTA，不再开放网络侧的固件写入通道）

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
   默认走 USB 串口（`upload_protocol = esptool`），端口由 PlatformIO 自动探测；
   不稳定时可显式指定 `upload_port`，或把 `upload_speed` 降到 115200。
4. 上传文件系统（Web 配置页面与样式）：
   ```
   pio run -t uploadfs
   ```
   `platformio.ini` 中已设置 `board_build.filesystem = littlefs`。

### 配置热点（AP）的开关逻辑
- 上电即开启热点 `aprs-tracker`（密码见 `src/main.cpp` 顶部 `AP_PASS` 宏）；v0.11 起内置强制门户
  DNS，手机连上后通常会自动弹出配置页，也可手动访问 `http://192.168.4.1/`；
- **成功连上你配置的 WiFi 后，热点被真正关闭**：调用 `WiFi.enableAP(false)` 从 opmode 摘掉 AP 位，
  AP 接口、DHCP 服务器与信标一并停止 —— 不是只把 SSID 藏起来；
- **WiFi 断开超过 30 秒，热点会自动重开**，可直接连上去重新填写 WiFi 参数；
- 断线期间设备会定期尝试重连（`WIFI_RETRY_PERIOD` 10 秒；配置热点开着时放宽到 30 秒）；
  关闭热点后 60 秒内（`AP_OFF_COOLDOWN`）即使断线也不重开，
  用于吸收切换 opmode 造成的短暂掉线，避免"关了又开"来回抖动；
- **自适应退避**：关闭热点后会观察 15 秒（`AP_OFF_CHECK_WINDOW`）确认 STA 是否被带下线。
  若连续 2 次关完就掉线，判定本板切换 opmode 会断网，此后保持热点常开，不再尝试关闭
  （首页显示 `ON (192.168.4.1, forced)`）；
- 热点关闭后，仍可通过设备在局域网中的 IP 访问配置页（串口日志会打印该 IP，
  首页也会显示热点当前是 `ON` 还是 `OFF`）。

> **为什么关闭要动 opmode？** ESP8266 的 `WiFi.softAPdisconnect(false)` 在 core 里只是把
> `softap_config.ssid` 置空，等同于隐藏 SSID，AP 仍在运行；而 SDK 头文件没有导出
> `wifi_softap_stop()`，不存在"只停 AP、不动 STA"的接口。因此真正关闭只能走
> `WIFI_AP_STA → WIFI_STA`，代价是切换瞬间 STA 可能掉线数秒后自动重连。

若热点已关闭且你想重新进入配置模式，可直接断开设备所在的 WiFi（或按一下复位），
30 秒后热点会重新出现。

### 配置页响应速度（v0.11 优化）
ESP8266 单核单线程，`server.handleClient()` 每次 `loop()` 只跑一次，**任何阻塞都会
直接变成页面卡顿**。v0.11 之前打开配置页经常要等十几秒，原因与处理：

| 位置 | v0.11 之前 | 现在 |
|---|---|---|
| `setup()` 等 WiFi | `while(WiFiMulti.run() != WL_CONNECTED)` 死等，连不上就永远进不了 `loop()` | 限时 20 秒（`WIFI_CONNECT_TIMEOUT`），超时先进入配置模式 |
| `loop()` 判断联网 | 每轮调阻塞的 `WiFiMulti.run()`，未连接时每次最多 10 秒 | 改用非阻塞的 `WiFi.status()`，重连交给 `manageWifi()` 限频 |
| 重连单次超时 | 每 AP 5 秒 × 2 = 最多 10 秒 | 压到 `WIFI_CONNECT_TRY_MS`（2 秒） |
| 各处 `delay()` | 纯阻塞，等待期间完全不响应 HTTP | 统一改 `smartDelay()`，等待中继续喂 GPS 并处理 Web 请求 |
| 闪灯 `delay(150)`×10 | 阻塞 1.5 秒 | 改 `smartDelay()` |

同时新增 `DNSServer` 强制门户：手机连热点后自动弹出配置页，也避免手机做连通性探测
时因无 DNS 而长时间等待。

> **仍存在的瓶颈**：GPS 用的是 `SoftwareSerial`（D6/D7，9600bps）。软件串口靠 CPU 逐位
> 采样，会持续占用资源、影响 WiFi 响应。想进一步提速可把 GPS 改接硬件串口，
> 或把波特率从 9600 提到 38400/115200（需同步改 `GPSBaud` 与 GPS 模块配置）。

## Web 配置流程
1. 手机/电脑连接 `aprs-tracker` 热点。
2. 浏览器打开 `http://192.168.4.1/`。
3. 依次配置：
   - **APRS config（必填）**：呼号（如 `BH9FXK-5`）、APRS passcode、注释、自定义信息、APRS-IS 服务器（如 `asia.aprs2.net`）、符号、SmartBeacon 参数。
     **各字段已预填推荐默认值**（见 `src/main.cpp` 的 `DEF_*` 宏），首次配置只需改呼号与
     passcode 即可直接保存；字段下方有灰色小字说明取值范围；
     若把关键字段清空后保存，固件会自动回落到默认值，避免设备因缺参数而不发信标。
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
- AP 密码硬编码在 `src/main.cpp` 顶部 `AP_PASS` 宏，**请按需修改**；热点在连上 WiFi 后会真正关闭，
  仅在断线超时后重开，暴露窗口有限。
- 配置文件（含 Wi-Fi 密码、APRS passcode）以明文存储在 LittleFS，**暂无任何接口可远程读取**。
- Web 配置接口默认无认证，连上同一 Wi-Fi 即可读写配置，请勿在不可信网络长期使用。
- 位置数据通过明文通道传输（APRS-IS 14580 本就明文；Traccar 默认 HTTP）。

### 已修复的安全问题
| 版本 | 问题 | 处理 |
|------|------|------|
| v0.7 | ArduinoOTA 无密码，同网段可任意刷入固件 | 移除 ArduinoOTA，固件改为 USB 串口本地烧录 |
| v0.7 | `/dl` 接口用 `server.arg(0)` 直接 `LittleFS.open`，可下载 `/wifis.txt`、`/aprs.txt` 等明文凭据文件 | 删除该路由与 `httpDownload()`（前端从未引用） |
| v0.6 | `positionReportWithAltitude()` 缓冲区仅 64 字节，拼接无长度校验 | 扩至 160 字节 |
| v0.10 | AP 关闭实际只置空了 SSID（等于隐藏热点），AP 接口、DHCP 与射频仍在运行，并未真正关闭 | 改用 `WiFi.enableAP(false)` 从 opmode 摘掉 AP 位，真正关闭 AP 接口 |

## 许可
本项目采用 [MIT License](./LICENSE)。
