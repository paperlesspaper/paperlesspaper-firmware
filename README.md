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
*   **BLE Advertising**: The device advertises via BLE for provisioning. Advertising restarts automatically after a client disconnects.
*   **Deep Sleep**: The device enters deep sleep to save power. It wakes up via:
    *   Timer (configurable via MQTT).
    *   Accelerometer (motion).
    *   Button press.

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
