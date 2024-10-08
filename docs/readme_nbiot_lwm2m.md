# Pinnacle 100 NB-IoT LwM2M Out of Box Demo

## Table of Contents

1. **[Introduction](#introduction)**
2. **[Resources](#resources)**
3. **[Prerequisites](#prerequisites)**
4. **[Setup](#setup)**
5. **[Using the Demo](#using-the-demo)**  
   [Auto Commissioning](#auto-commissioning)  
   [Manual Commissioning](#manual-commissioning)  
   [View Cloud Data](#view-cloud-data)  
6. **[User Defined Server](#user-defined-server)**
7. **[LED Behavior](#led-behavior)**
8. **[Building the Firmware](#building-the-firmware)**
9. **[Development](#development)**

## Introduction

When using NB-IoT, we do not recommend using the TCP protocol due to the network latencies inherent to NB-IoT. UDP is recommended for NB-IoT data transfer.
In this configuration, the MG100 is configured to use LwM2M and DTLS with pre-shared keys.  [LightweightM2M (LwM2M)](http://www.openmobilealliance.org/wp/Overviews/lightweightm2m_overview.html) is a protocol that runs on top of [CoAP](https://coap.technology/).

A [Leshan Server](https://www.eclipse.org/leshan/) is used to display information about the MG100 and a connected BL654 BME280 sensor.

The software version, model, manufacturer, serial number, and current time can be read. If a BME280 sensor is connected, then temperature, humidity, and pressure can be read. In addition, the green LED on the DVK can be turned on and off using the light control object.

```
                XXXXX
              XXX   XXX
    XXXXX   XX        XX
 XXX    XX XX          X XXXXXXX
 X        XX            X       XX
X                                X
 X       Leshan LwM2M Server      X
 XX                              X
  XXXXXXX                       XX
        XX          XXXX    XXXX
         XXX      XXX   XXXX
           XXXXXXX
               ^
               +
            NB-IoT (LwM2M with DTLS)
               +
               v
     +---------+----------+
     |                    |
     |       MG100        |
     |                    |
     +---------+----------+
               ^
               +
              BLE
               +
               v
    +----------+------------+
    |                       |
    |  BL654 BME280 Sensor  |
    |                       |
    +-----------------------+

```

## Resources

- [MG100 product page](https://www.lairdconnect.com/iot-devices/iot-gateways/sentrius-mg100-gateway-lte-mnb-iot-and-bluetooth-5)

## Prerequisites

The following are required to use the MG100:

- An activated SIM card. The Truphone SIM card that comes with the DVK does not currently support NB-IoT. If you have purchased a MG100 in Europe, then please contact [sales](mailto:sales@lairdconnect.com) to obtain a SIM card that is NB-IoT capable.
- MG100 programmed with NB-IoT/LwM2M MG100 firmware.
- Laird Pinnacle Connect app installed on a mobile device
  - [Android app](http://play.google.com/store/apps/details?id=com.lairdconnect.pinnacle.connect)
  - [iOS app](https://apps.apple.com/us/app/laird-pinnacle-connect/id1481075861?ls=1)

## Setup

To set up the MG100, follow these steps:

1. Ensure an *activated* SIM card is inserted into the MG100 SIM slot.
2. On your phone, launch the Pinnacle mobile app and follow the on-screen prompts.

## Using the Demo
If the Laird Connectivity Leshan server is used, then the MG100 should be configured to something other than its default settings. This allows multiple MG100 devices to connect to the same server.

### Auto Commissioning

1. Open the mobile app, connect to your device and go to the LwM2M Settings page.
2. Click the Auto-commission device button.  This will automatically add the MG100 as a unique device to the Leshan LwM2M server.

![Auto-commission device](images/lwm2m_auto_commission.png)  
_Auto-commission device_

3. Skip to [View Cloud Data](#view-cloud-data) for instuctions on interacting with your device.

### Manual Commissioning

1. Open [Laird Connectivity Leshan web page](http://uwterminalx.lairdconnect.com:8080/#/clients).
2. Go to the Security page.
3. Press "Add new client configuration".

![Leshan add device](images/leshan_add_device.png)  
_Leshan add device_

4. Using the Pinnacle Connect mobile app, read the model number from the Device Information page.

![Model number](images/app_model_number.png)  
_Model number_

5. With the mobile app, read the IMEI from the Cellular Settings page.

![IMEI](images/app_imei.png)  
_IMEI_

6. Enter <model number>_<imei> into the Client endpoint field on the webpage.  For example, mg100_354616090287629.
7. Generate a new private shared key using the LwM2M Settings page in the mobile app.
8. Copy the value into the Key field on the webpage.
8. Set and save the Client ID using the mobile app (don't reuse a name that is already present on the server).
9. Put the same name into the Identity field on the web page.

![LwM2M settings](images/lwm2m_manual_commission.png)  
_LwM2M settings_

![Leshan device config](images/leshan_device_config.png)  
_Leshan device config_

10. Reset the modem using the mobile app (Power Settings page) or reset button (SW5 NRF_RESET).

### View Cloud Data
From the [clients page](http://uwterminalx.lairdconnect.com:8080/#/clients), click on your device once it is connected to interact with it.
  * Make sure to set the Response Timeout to something greater than 5s at the top of the page. 

![Leshan response timeout](images/leshan_response_timeout.png)  
_Leshan response timeout_

  * Click `Read` at the top of an object to read all child values.  In some cases, not all child values are supported.

![Leshan read object](images/leshan_read_object.png)  
_Leshan read object_

  * The Light Control object can be used to control the green LED on the dev board. Write `true` or `false` to the On/Off node.

![Leshan light control](images/leshan_light_control.png)  
_Leshan light control_

## User Defined Server
In addition to the steps in the [Using the Demo](#using-the-demo) section, the Peer URL must be set using the mobile app.

## LED Behavior

The Blue LED blinks once a second when the MG100 is searching for a BL654 sensor. When it finds a sensor and successfully connects to it, the LED remains on.

The Green LED can be controlled via the LwM2M server Light Control object.

The Red LED blinks when the MG100 is searching for a cellular network. It remains on and does not blink when connected to a network. If there is an error with the SIM card or network registration, then the LED remains off.

## Building the Firmware

The firmware can be built to work with or without the mcuboot bootloader. Building without mcuboot is faster and easier for development and debug, but gives up the ability to update the Zephyr app via UART or BLE.

Issue these commands **from the mg100_firmware directory**.

Build without mcuboot:
```
# Linux and macOS

rm -f mg100/pm_static.yml && west build -b mg100 -d build/mg100_lwm2m mg100

# Windows

del mg100\pm_static.yml && west build -b mg100 -d build\mg100_lwm2m mg100
```

> **Note:** When switching between builds with or without mcuboot, be sure to delete the build directory before building.

Build with mcuboot:
```
# Linux and macOS

cp ../modules/zephyr_lib/mcuboot_config/pm_static.pinnacle100.yml mg100/pm_static.yml && west build -b mg100 -d build/mg100_lwm2m mg100 -- -DOVERLAY_CONFIG="${PWD}/mg100/overlay_lwm2m_dtls.conf ${PWD}/../modules/zephyr_lib/mcumgr_wrapper/config/overlay-mcuboot.conf" -Dmcuboot_CONF_FILE="${PWD}/../modules/zephyr_lib/mcuboot_config/mcuboot-qspi.conf ${PWD}/../modules/zephyr_lib/mcuboot_config/overlay-disable-hl7800-modem.conf ${PWD}/../modules/zephyr_lib/mcuboot_config/mcuboot-serial-pinnacle100_dvk.conf ${PWD}/../modules/zephyr_lib/mcuboot_config/disable-slot0-validate.conf"

# Windows

copy ..\modules\zephyr_lib\mcuboot_config\pm_static.pinnacle100.yml mg100\pm_static.yml && west build -b mg100 -d build\mg100_lwm2m mg100 -- -DOVERLAY_CONFIG="%CD%\mg100\overlay_lwm2m_dtls.conf %CD%\..\modules\zephyr_lib\mcumgr_wrapper\config\overlay-mcuboot.conf" -Dmcuboot_CONF_FILE="%CD%\..\modules\zephyr_lib\mcuboot_config\mcuboot-qspi.conf %CD%\..\modules\zephyr_lib\mcuboot_config\overlay-disable-hl7800-modem.conf %CD%\..\modules\zephyr_lib\mcuboot_config\mcuboot-serial-pinnacle100_dvk.conf %CD%\..\modules\zephyr_lib\mcuboot_config\disable-slot0-validate.conf"
```

After building the firmware, it can be flashed with the following command:
```
# Linux and macOS

west flash -d build/mg100_lwm2m

# Windows

west flash -d build\mg100_lwm2m
```

If the firmware was built with mcuboot, `west flash` will program merged.hex which contains the mcuboot bootloader and app combined. 

## Development
See [here](development.md) for help on getting started with custom development.
