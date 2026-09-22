# E-Paper ESP32-C6 Firmware

Firmware for an ESP32-C6 based E-Paper display device, featuring BLE provisioning, AWS IoT connectivity, and OTA updates. We use the firmware in our [paperlesspaper OpenPaper 7](https://paperlesspaper.de/en/buy-7-inch-epaper-picture-frame). Check [paperlesspaper Github](https://github.com/paperlesspaper) for hardware source files.

![paperlesspaper preview](https://paperlesspaper.de/_next/image?url=https%3A%2F%2Fres.cloudinary.com%2Fwirewire%2Fimage%2Fupload%2FIMG_5826-Bearbeitet.jpg.jpg&w=3840&q=75)

## Hardware Requirements

*   **Microcontroller**: ESP32-C6-DevKitM-1
*   **Display**: Spectra 6 7.3 (EL073TF1)
*   **Sensors**: KXTJ3-1057 Accelerometer
*   **Other**: Battery, Charger circuit (see Hardware Settings below)

## Software Requirements

*   **IDE**: Visual Studio Code
*   **Extension**: PlatformIO
*   **Framework**: Arduino (via PlatformIO)

## License

This project is released under the GNU General Public License v3.0 (GPL-3.0). See the [`LICENSE`](LICENSE) file for the full license text.

## Installation & Setup

1.  **Clone the Repository**
    ```bash
    git clone <repository-url>
    cd epaper-espc6-firmware
    ```

2.  **Environment Configuration**
    Create a `.env` file in the root directory with the following variables:
    ```ini
    ENV_OTA_URL="https://your-ota-server.com/firmware.bin"
    ENV_OTA_URL_DEV="https://your-dev-ota-server.com/firmware-dev.bin"
    ENV_WIFI_PW_DEPLOY="default_wifi_password"
    ENV_WIFI_SSID_DEPLOY="default_wifi_ssid"
    ENV_AWS_IOT_ENDPOINT="your-aws-iot-endpoint.iot.region.amazonaws.com"
    ```

3.  **AWS IoT Certificates & SPIFFS**
    The device requires AWS IoT certificates to connect to the cloud service. These are stored in the SPIFFS filesystem.
    
    > ℹ️ **Important Note on Certificates:** Device-specific AWS IoT certificates are not included in this open repository for security reasons. Certificates are **exclusively available directly via paperlesspaper Customer Support**.
    
    *   **Naming Convention**: The certificates must be named using the device's unique ID, which is `epd7-` followed by the MAC address (hex, uppercase, no colons).
        *   Example MAC: `A0:B1:C2:D3:E4:F5` -> UID: `epd7-A0B1C2D3E4F5`
        *   Private Key: `epd7-A0B1C2D3E4F5.key`
        *   Certificate: `epd7-A0B1C2D3E4F5.crt`
    
    *   **Upload via PlatformIO**:
        1.  Create a `data` folder in the project root if it doesn't exist.
        2.  Place your renamed `.key` and `.crt` files in the `data` folder.
        3.  Run the PlatformIO task: `Platform` -> `Upload Filesystem Image`.

    *   **Upload via Web-UI**:
        1.  Open the Web Flasher UI (under **Firmware Update** -> **USB-Kabel (COM-Port)**).
        2.  Use the **"SPIFFS / Zertifikat per USB übertragen (.bin)"** button to flash a pre-packaged certificate `.bin` image obtained from Support.

    > **Warning**: These certificates are stored in the SPIFFS partition. If you change the partition table or erase the flash, the certificates will be lost, and the device will no longer connect to the cloud.

4.  **Build and Upload Firmware**
    *   Run the PlatformIO task: `General` -> `Upload`.

## 🌐 Web-UI & Tools

*   **Web Serial Flashing**: Flash official/custom firmware `.bin` files or SPIFFS certificate images directly in your browser over USB-C.
*   **Web Serial Debug Monitor**: Open a live Web Serial terminal at 115200 baud directly in the Web UI to monitor UART debug logs without external software.
*   **Web-BLE Image Upload**: Wirelessly preview, dither (Floyd-Steinberg, Atkinson, Sierra 2, etc.), auto-optimize, and send images directly to the frame.

## Usage Limits & Cloud Connectivity

*   **AWS IoT**: This firmware heavily relies on AWS IoT Core for activation, status updates, and image retrieval. Ensure your AWS account is set up and limits/costs are monitored.
*   **BLE Advertising**: The device advertises via BLE for provisioning and OTA updates. Advertising restarts automatically after a client disconnects.
*   **Deep Sleep**: The device enters deep sleep to save power. It wakes up via:
    *   Timer (configurable via MQTT).
    *   Accelerometer (motion).
    *   Button press.

## 📶 Bluetooth Low Energy (BLE) Interface

The ESP32-C6 firmware exposes a GATT server (implemented via NimBLE) primarily for **WiFi onboarding / provisioning** and **OTA firmware updates**.

### Device Advertising & Discovery
* **Advertised Device Name**: Matches the unique device ID (e.g. `epd7-A0B1C2D3E4F5` or `epd13-A0B1C2D3E4F5`).
* **Advertised Service UUIDs**:
  * Device Data Service (`7f74170e-7b0e-11ed-a1eb-0242ac120002`)
  * WiFi Configuration Service (`0515c086-7b0c-11ed-a1eb-0242ac120002`)
  * E-Paper Settings / Firmware Update Service (`10000000-0000-0000-0000-000000000001`)
* **When BLE is Active**:
  * On first boot or when no valid WiFi credentials exist in flash.
  * When WiFi connection fails and the device was woken up via button (`buttonWake`).
  * BLE terminates automatically after successful WiFi connection or when the timeout is reached (device returns to deep sleep).

---

### GATT Services & Characteristics Overview

| Service | Service UUID | Characteristic | Characteristic UUID | Properties | Format / Data Type | Description & Function |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Device Data Service** | `7f74170e-7b0e-11ed-a1eb-0242ac120002` | **WiFi Connected Status** | `4c578d4c-7b0e-11ed-a1eb-0242ac120002` | `READ` | `uint8` (`0` or `1`) | Indicates the connection state (`1` = connected, `0` = disconnected/connecting). Polled during provisioning to confirm success. |
| | | **WiFi Scan Results** | `5131a3fc-7b0e-11ed-a1eb-0242ac120002` | `READ` | UTF-8 String (Descriptor `2904`) | List of scanned networks in the format `SSID´RSSI´´SSID´RSSI´´...` (max. ~460 bytes). |
| **WiFi Configuration Service** | `0515c086-7b0c-11ed-a1eb-0242ac120002` | **WiFi SSID** | `090b0ef2-7b0d-11ed-a1eb-0242ac120002` | `READ`, `WRITE` | UTF-8 String (Descriptor `2904`) | Target WiFi SSID. Writing sets the SSID for connection and flash storage (max. 35 chars). |
| | | **WiFi Password** | `a62eed84-7b0d-11ed-a1eb-0242ac120002` | `READ`, `WRITE` | UTF-8 String (Descriptor `2904`) | Target WiFi Password (max. 65 chars). Writing both SSID and password triggers immediate WiFi connection. |
| **E-Paper & Firmware Update Service** | `10000000-0000-0000-0000-000000000001` | **Upload Data (OTA Chunks)** | `10000003-0000-0000-0000-000000000001` | `WRITE`, `WRITE_NR` | Binary (`[4-Byte CRC32 LE] + [Payload]`) | Firmware OTA data chunks. Each chunk begins with a 4-byte little-endian CRC32 of the payload, checked before copying into the 19.2 KB RAM buffer. |
| | | **Upload Command / Status** | `10000004-0000-0000-0000-000000000001` | `READ`, `WRITE` | **READ**: `uint16` LE (Buffer Pos or `0xFFFF` on error)<br>**WRITE**: String Command | Controls OTA update lifecycle via commands: `START_FW`, `FLUSH`, `CLEAR`, `END_FW`. |

---

### Communication Workflows

#### 1. WiFi Provisioning Workflow
1. **Scan & Discover**: Scan for BLE devices with name prefix `epd7-` or `epd13-`.
2. **Connect**: Establish GATT connection to the device.
3. **(Optional) Read Nearby Networks**: Read characteristic `5131a3fc-7b0e-11ed-a1eb-0242ac120002` to retrieve the list of detected SSIDs and RSSI values (separated by `´´`).
4. **Write Credentials**:
   * Write target SSID as UTF-8 string to `090b0ef2-7b0d-11ed-a1eb-0242ac120002`.
   * Write target Password as UTF-8 string to `a62eed84-7b0d-11ed-a1eb-0242ac120002`.
5. **Verify Connection**: Poll the status characteristic `4c578d4c-7b0e-11ed-a1eb-0242ac120002`. When it reads `1`, WiFi is verified and saved permanently into flash memory. The ESP32 will disconnect and shut down BLE.

*(A ready-to-use Python client for this workflow is provided in [`testbench/ble_provisioner.py`](testbench/ble_provisioner.py).)*

#### 2. BLE OTA Firmware Update Workflow
1. **Start Update**: Write command string `"START_FW"` to `10000004-0000-0000-0000-000000000001`. This initializes the OTA partition update and allocates an internal 19,200-byte RAM buffer.
2. **Stream Binary Chunks**: Send chunks to `10000003-0000-0000-0000-000000000001`. Each packet must be prefixed with a 4-byte CRC32 (little-endian) of the chunk payload. The firmware verifies the CRC before accepting the bytes into the buffer.
3. **Flush Buffer**: Periodically send command string `"FLUSH"` to `10000004-...` before the 19.2 KB buffer fills up, writing buffered bytes to flash.
4. **Finish Update**: Send command string `"END_FW"` to `10000004-...`. Remaining bytes are written, the firmware binary is validated, and the ESP32 automatically reboots into the new firmware. If validation fails, reading `10000004-...` returns `0xFFFF`.

## Memory Map (EEPROM/Flash)

*   `0-39`: WiFi Name
*   `40-105`: WiFi Password
*   `140`: Reconnect Count
*   `150`: File Version
*   `160`: Activated Flag
*   `170`: Activation Counter
*   `190`: Display Revision Store
*   `200`: WiFi Lost State
*   `210`: Sleep Time
*   `220`: Dispay Orientation Store
*   `500+`: Settings Store

## Hardware Settings

*   **Charger**: Safety TMR 4h, 4-cell intermittent.
*   **Reset**: 5+ presses
*   **OTA Force (Dev Firmware)**: 3-4 presses
*   **Boot Mode**: Hold the small button, short press the reset button (big button) while holding the small button, then release the small button.
