# Arduino CLI + ESP32 + micro-ROS Arduino Guide

## Architecture

``` text
Laptop (VS Code Remote SSH)
        |
        v
Raspberry Pi 5
   |- Arduino CLI
   |- ROS 2 Jazzy
   |- micro_ros_agent
   `- USB-connected ESP32
```

## Install Arduino CLI

``` bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/
arduino-cli version
arduino-cli config init
arduino-cli core update-index
```

## Install ESP32 Board Support

``` bash
arduino-cli config set board_manager.additional_urls \
https://espressif.github.io/arduino-esp32/package_esp32_index.json

arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli core list
```

## Detect ESP32

``` bash
arduino-cli board list
```

## Create a Sketch

``` bash
mkdir ~/arduino_projects
cd ~/arduino_projects
arduino-cli sketch new blink
```

## Compile

``` bash
arduino-cli compile --fqbn esp32:esp32:esp32 blink
```

## Upload

``` bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 blink
```

## Library Management

Update index:

``` bash
arduino-cli lib update-index
```

Search:

``` bash
arduino-cli lib search ArduinoJson
```

Install:

``` bash
arduino-cli lib install ArduinoJson
```

List installed:

``` bash
arduino-cli lib list
```

## Install micro_ros_arduino from Library Manager

``` bash
arduino-cli lib search micro_ros_arduino
arduino-cli lib install micro_ros_arduino
```

Or a specific version:

``` bash
arduino-cli lib install micro_ros_arduino@3.0.0-iron
```

Verify:

``` bash
arduino-cli lib list | grep micro_ros
```

## Install micro_ros_arduino from ZIP

``` bash
wget https://github.com/micro-ROS/micro_ros_arduino/archive/refs/heads/main.zip
arduino-cli lib install --zip-path ./main.zip
```

## Create micro-ROS Sketch

``` cpp
#include <micro_ros_arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
}
```

## Compile micro-ROS Sketch

``` bash
arduino-cli compile --fqbn esp32:esp32:esp32 microros_pub
```

## Flash micro-ROS Sketch

``` bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 microros_pub
```

## Run micro-ROS Agent on Raspberry Pi

``` bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Verify ROS 2

``` bash
ros2 topic list
ros2 topic echo /my_topic
```

## Daily Workflow

``` bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 .
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
ros2 topic list
```



Most likely something is already connected to the ESP32 serial port:

* `micro_ros_agent`
* `screen`
* `minicom`
* Arduino Serial Monitor
* VS Code serial monitor
* another upload process

### Check who is using the port

Run:

```bash
sudo lsof /dev/ttyAMA10
```

or:

```bash
sudo fuser -v /dev/ttyAMA10
```

Example output:

```text
/dev/ttyAMA10: 12345
```

or

```text
COMMAND   PID USER
screen   12345 lucy
```

---

### Kill the process

If you get a PID:

```bash
kill 12345
```

If it doesn't stop:

```bash
kill -9 12345
```

Or directly with `fuser`:

```bash
sudo fuser -k /dev/ttyAMA10
```

---

### Find common serial programs

```bash
ps aux | grep -E "micro_ros_agent|screen|minicom|picocom|putty"
```

Kill the offending process:

```bash
pkill micro_ros_agent
```

or

```bash
pkill screen
```

---

### Verify the port is free

```bash
sudo lsof /dev/ttyAMA10
```

No output means the port is free.

---

### Check if the ESP32 changed ports

Sometimes after reconnecting, the ESP32 appears as:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

or

```bash
arduino-cli board list
```

The board may no longer be on `/dev/ttyAMA10`.

---

Run this and paste the output:

```bash
sudo lsof /dev/ttyAMA10
```

That will tell us exactly what's holding the port.
