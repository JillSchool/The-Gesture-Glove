# RX_BLE.py

## Overview

The `rx_ble.py` script facilitates communication between a BLE-enabled motion detection glove and a Webots simulation. It is designed to handle BLE device scanning, establish a TCP connection with Webots, and manage data transmission between the BLE device and the Webots simulation.

## Features

- Scans for available BLE devices.
- Connects to a Webots simulation via a TCP socket.
- Receives data from a BLE-enabled glove and forwards it to Webots.
- Sends test messages to Webots to verify the connection.
- Handles BLE disconnection events gracefully.

## Requirements

- Python 3 (tested with Python 3.12)
- Install the required Python packages listed in `requirements.txt`.

## Constants

- `GLOVE_ESP32_BLE_ADDRESS`: BLE address of the glove ESP32 device.
- `GLOVE_ESP32_BLE_CHARACTERISTIC_UUID`: UUID of the BLE characteristic to read data from.
- `WEBOTS_TCPIP_IP_ADDRESS`: IP address for the Webots TCP connection.
- `WEBOTS_TCPIP_PORT`: Port for the Webots TCP connection.

## Global Variables

- `webots_reader`: Reader object for Webots TCP connection.
- `tx_writer`: Writer object for Webots TCP connection.
- `cmd_prev`: Stores the previous command received from BLE.

## Functions

### `main()`

The main function manages BLE and Webots connections. It initializes BLE scanning, connects to Webots, and handles BLE data reception.

### `scan_ble_devices()`

Scans for available BLE devices and prints their details.

### `connect_to_webots()`

Establishes a TCP connection to the Webots simulation.

### `close_connection()`

Closes the TCP connection to Webots.

### `cleanup()`

Resets global variables and events for cleanup.

### `test_webots_connection(message="P")`

Sends a test message to Webots to verify the connection.

### `rx_from_ble()`

Receives data from the BLE device and forwards it to Webots.

### `tx_to_webots(message)`

Sends a message to Webots via the TCP connection.

### `loop_iteration(use_scan=False, use_connect=True, use_test=False, use_rx=True)`

Performs a single iteration of the main loop, handling BLE scanning, Webots connection, test message sending, and BLE data reception.

## Usage

1. Ensure Python 3.7 or higher is installed.
2. Install the required Python packages using `requirements.txt`:

   ```bash
   pip install -r requirements.txt
   ```

3. Run the script using Python:

   ```bash
   python rx_ble.py
   ```

4. The script will scan for BLE devices, connect to Webots, and handle data transmission.

## Notes

- Update the `GLOVE_ESP32_BLE_ADDRESS` constant with the correct BLE address of your glove device.
- Ensure Webots is running and accessible at the specified IP and port.

## License

This script is provided under the MIT License. See the LICENSE file for details.
