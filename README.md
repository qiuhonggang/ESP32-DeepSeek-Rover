# ESP32-S3 DeepSeek 四驱小车

ESP32-S3 N16R8 是唯一车载控制器，双 INMP441 负责唤醒、语音输入和左右声源定位，MAX98357A 驱动 3070 喇叭。ESP32 通过 HTTPS 直接访问 DeepSeek ASR、Flash 和 TTS。四路电机控制、声学前处理、测距急停、网格建图和寻路都在车上完成。

| 功能 | 当前实现 |
| --- | --- |
| 本地唤醒 | ESP-SR WakeNet9 / WakeNet10 共 20 个预置词，默认「嗨小象」。网页选择一个，重启后按所选词唤醒 |
| 双麦和插话 | 双声道采样、左右 DOA、I²S 播放参考回声抵消。播报时插话可取消旧轮次 |
| 云端语音 | 16 kHz 单声道 WAV → DeepSeek ASR → `deepseek-flash` → DeepSeek TTS。24 kHz 播报音频流式转换到现有 16 kHz 音频链路 |
| 简单 harness | 本轮编号、ASR/Flash/TTS 阶段、设备状态和地图地点注入、动作校验、取消和超时处理 |
| 记忆 | 使用 NVS 保存 12 轮已播完对话，最近 6 轮作为上下文。给 4 岁小朋友的普通话短句，支持语音保存的偏好和网页资料 |
| 四驱 | 两块 DRV8833 驱动四个通道，左右两侧差速。网页可单轮点动、调四轮极性、架空测试 |
| 建图与寻路 | 单颗前向超声波配合停转扫描，21×21 占据网格，BFS 绕行、已知区域导航、命名地点、返回起点 |
| 本地保护 | 开机锁电机，动作限时，前向测距过近或失效停车。运行中断导致位姿未知时停止导航 |

**导航属于实验性低速功能。** 车靠定时 PWM 估算单格位移和 90° 转角，转向时的打滑、地面、负载和电压都会让估计位置漂移。单颗超声波只能测朝向的一束距离，侧面、后方及地面落差超出覆盖范围。地图适合在平坦、封闭、有清晰回波的小场地验证。丢回波会拦截前进，开阔处可能因此停下。 [HC-SR04 资料](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) · [机器人里程计说明](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom/)

## 烧录

源码用 ESP-IDF 6.0.2 编译。本地预编译包保存在 `release/`，包含程序、分区表、bootloader 和唤醒模型。**首次烧录请刷全部四段**，只刷应用会缺少 WakeNet 模型。配置、地图和记忆存放在独立 NVS 分区，更新同项目固件可以沿用这组分段偏移。

解压烧录包，进入解压目录。将 `PORT` 替换为你的串口，例如 `/dev/cu.usbmodemXXXX` 或 `/dev/cu.usbserial-XXXX`。

```bash
python -m pip install "esptool>=5,<6"
python -m esptool --chip esp32s3 -p PORT -b 460800 write-flash @flash_args
```

从 GitHub 获取源码并编译。已有本地项目时，直接在项目目录中编译。

```bash
git clone git@github.com:qiuhonggang/ESP32-DeepSeek-Rover.git
cd ESP32-DeepSeek-Rover
source /path/to/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

**从 1.0.0 或 1.0.1 更新到 1.0.2** 时，用新烧录包的四段完整写入，包括扩容后的分区表和新 `srmodels.bin`。应用、模型和 NVS 的起始偏移沿用旧版，模型分区从 3 MiB 扩为 8 MiB。用包内命令分段写入并保留 NVS，已有的唤醒词设置、Wi-Fi、DeepSeek Key、地图和对话记忆均可沿用。

串口速率为 115200。若开发板有 USB 和 UART 两个接口，首次接线可选带 USB 转串口的 UART 接口读日志，或使用板上 USB Serial/JTAG 口。串口显示 `WakeNet ready: 嗨小象 (...)` 或所选词的名称，代表模型初始化完成。 [ESP32-S3 DevKitC 手册](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html) · [WakeNet 文档](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/wake_word_engine/README.html)

## 首次配网与语音

1. 先断开电池和电机动力，按 [接线方案](WIRING.md) 验证供电，用 USB 给主控上电。查看串口打印的 `DeepSeekRover-XXXX` 和控制台口令。
2. 用电脑或手机浏览器加入该热点，打开 `http://192.168.4.1`。热点的 WPA2 密码和网页控制台口令使用串口显示的同一个值。浏览器只用来配置和调试。
3. 填写能联网的 2.4 GHz Wi-Fi 和自己的 DeepSeek API 密钥，保存重启。接回同一局域网，打开串口显示的 ESP32 IP。正常联网后关闭配网热点，断开后约 12 秒重新开放。网页上「正在同步时钟」消失后可发起 HTTPS 请求。
4. 点「喇叭测试音」确认功放，查看左右麦 RMS 是否随声音变化。点「手动听一句」，确认转写和回复，再说「嗨小象」，稍停一下后说「你叫什么名字」。说完等约 0.7 秒结束录音。播报时可直接插话，具体识别效果需按实际装配调音量、门限和参考延迟。
5. 任意时候用网页「急停并锁定」停车，手动重新启用后才可下发电机动作。本地识别到唤醒词也会立即暂停车体，语音「停止」在完成 ASR 后再次下达停车。

DeepSeek 路径固定为官方域名 `api.deepseek.com`。设置密钥时用已加密的配网热点或自己信任的局域网，网页管理端使用 HTTP。固件使用证书校验连接 DeepSeek，密钥写入设备 NVS，不出现在源码、包或网页读取接口内。已唤醒的录音和允许使用的对话上下文会发给 DeepSeek。 [ASR](https://api-docs.deepseek.com/zh-cn/guides/asr/) · [Flash JSON 输出](https://api-docs.deepseek.com/zh-cn/guides/json_mode/) · [TTS](https://api-docs.deepseek.com/zh-cn/guides/tts/)

## 选择本地唤醒词

进入网页「Wi-Fi、DeepSeek 与参数」，在「本地唤醒词」里任选一个，再点「保存并重启」。启动后「唤醒与回声」会显示当前生效的选项。固件只启用所选的一个模型。更换唤醒词时，密码和 API Key 字段留空即可。

| 选项 | 唤醒词 |
| --- | --- |
| 原有 4 个 | 嗨小象、嗨喵喵、你好小智、嗨，乐鑫 |
| 新增 WakeNet9 | 小鸭小鸭、小鹿小鹿、小龙小龙、小冰小冰、小珈小珈、嗨小欧、小酥肉、你好星宝、你好东东、你好小安、喵喵同学、小美同学 |
| 新增 WakeNet10 | 双儿、涂涂你好、豆包豆包、DeepSeek |

先在车停稳且喇叭安静时试念完整唤醒词，再在平常使用的位置试。孩子真实发音的唤醒率和误触发需用装好的车验证。 [乐鑫预置词列表](https://github.com/espressif/esp-sr/blob/master/wakeword_list.md)

## 四轮校准与建图

1. 架空四轮，把挡板放在超声波前约 40–80 cm，确认网页测距有效和车头方向。启用电机，按「左前、右前、左后、右后」逐个点动，确认正确轮子转动，借助四轮极性选项统一前进方向。
2. 放到平地，用胶带标记固定起点和初始朝向。用尺子量「标定走一格」的实际位移。默认格边长 25 cm，调整 PWM 和前进时间，使实际位移接近一格。分别标定左转、右转按键的 90° 时间，避免默认毫秒值被当成实测参数。
3. 把车放回起点。首次选「清空并设起点」，已有地图时选「重新对齐起点」，需复现原位置、朝向以及地面条件。将地图的「上」理解为你设置的方向。
4. 点「开始扫描建图」或语音说「开始建图」。小车在站点按四个方向转身测距，标记有足够前向余量的网格，在确认可通行的网格中规划下一步。默认要求移动前前方距离至少达到 **一格长度 + 急停距离 + 6 cm**。每步行驶中另外持续检测急停距离。路线前方变得不可通行时，车在当前格重算路线；若在移动途中急停，则需要回到原点重新对齐。
5. 停稳后可点击已知空闲格导航，或者把当前格命名为「客厅」等地点。用网页或语音「返回起点」「去客厅」开始导航，云端语音命令也会校验地图和电机状态。地图须点「保存地图」才会跨重启保留，命名地点时会同时保存。重启或手动点动后，车身位姿需要重新对齐。

地图中心是起点 `(10,10)`，行列从 0 到 20。默认 25 cm 时理论栅格覆盖约 5.25 × 5.25 m，实际覆盖取决于超声波回波、转角和轮胎打滑。单次运行最多行驶 32 格或 135 秒。可在地图上查看估计位置及前方测距。

## 常用语音指令

每轮先说网页上选定的唤醒词，默认「嗨小象」。稍停后说一整句，动作短句以表内示例为准。

| 想做什么 | 可以这样说 |
| --- | --- |
| 行走 | 前进、后退、左转、右转、扭一扭 |
| 按双麦方向转 | 朝我转一下 |
| 停车 | 停止、别动、急停 |
| 自动扫描 | 开始建图、开始巡游 |
| 地点操作 | 把这里记为客厅、去客厅、返回起点 |
| 保存一条偏好 | 记住我喜欢简短的回答 |
| 普通对话 | 介绍一下你自己 |

含否定、转述或连续多个动作的句子按对话处理。动作需要匹配当前语音短句和 Flash 返回的动作名，超过 12 秒的动作回复作废。部分转写词会因口音有所不同，网页可以看到原始转写。「停止」完成语音识别后由设备直接处理。后退只短程点动，前置传感器无法观察后方。唤醒词会先停下当前运动。单纯问答也会暂停巡游。

对话默认面向 4 岁小朋友。小车平时用一两句简单、温柔的普通话回答，一次只问一个问题，故障提示也用孩子能听懂的说法。保存记忆以播报完成为准，被插话的回复不会写入对话记录。关闭历史开关停止写入和传送历史轮次，单独保存的偏好及网页资料仍作为上下文。网页的「清空历史和语音偏好」清理这两项，网页资料独立编辑。

## 源码与检查

| 目录或文件 | 作用 |
| --- | --- |
| `main/audio.c`、`main/acoustics.c`、`main/wake_words.c` | I²S、回声消除、唤醒、DOA 和音频播放 |
| `main/cloud.c`、`main/audio_wire.c` | 三个 DeepSeek HTTPS 请求、WAV 文件和流式 PCM |
| `main/harness.c`、`main/intent.c` | 对话上下文、NVS 记忆和动作意图 |
| `main/motion.c`、`main/nav.c`、`main/grid.c` | 四轮控制、导航和地图规划 |
| `main/network.c`、`main/console.html` | 配网、网页状态与操作 |
| `tests/core.c` | 平台无关的语音格式、动作限制和规划回归检查 |

```bash
cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -I main tests/core.c \
  main/grid.c main/intent.c main/audio_wire.c main/wake_words.c -o /tmp/rover-core-tests
/tmp/rover-core-tests
```

已完成固件和唤醒模型构建。真机的 DeepSeek 请求、语音唤醒、四轮方向、供电抗压降和实际导航表现需要按以上顺序验证。
