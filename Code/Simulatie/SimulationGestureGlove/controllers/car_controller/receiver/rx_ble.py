"""
This module facilitates communication between a BLE-enabled motion detection glove and a Webots simulation.
It includes functions for scanning BLE devices, connecting to Webots via TCP, and handling BLE data.

Constants:
    GLOVE_ESP32_BLE_ADDRESS (str): BLE address of the glove ESP32 device.
    GLOVE_ESP32_BLE_CHARACTERISTIC_UUID (str): UUID of the BLE characteristic to read data from.
    WEBOTS_TCPIP_IP_ADDRESS (str): IP address for the Webots TCP connection.
    WEBOTS_TCPIP_PORT (int): Port for the Webots TCP connection.

Global Variables:
    webots_reader: Reader object for Webots TCP connection.
    tx_writer: Writer object for Webots TCP connection.
    cmd_prev (str): Stores the previous command received from BLE.
"""

import asyncio
from bleak import BleakScanner, BleakClient
from typing import Final

# Define constants for BLE and Webots connection
GLOVE_ESP32_BLE_ADDRESS_DEV: Final[str] = 'E0:5A:1B:E2:C1:C2' # Development Glove ESP32 BLE address
GLOVE_ESP32_BLE_ADDRESS_MAIN: Final[str] = '08:3A:F2:59:B8:D2' # Main Glove ESP32 BLE address
GLOVE_ESP32_BLE_ADDRESS: Final[str] = GLOVE_ESP32_BLE_ADDRESS_MAIN
GLOVE_ESP32_BLE_CHARACTERISTIC_UUID: Final[str] = 'beb5483e-36e1-4688-b7f5-ea07361b26a8' # Characteristic UUID
WEBOTS_TCPIP_IP_ADDRESS: Final[str] = 'localhost' # IP address for Webots TCP connection
WEBOTS_TCPIP_PORT: Final[int] = 10020 # Port for Webots TCP connection

# Global variables for Webots connection
webots_reader, tx_writer = None, None
cmds_prev: str = None # Store the previous command received from BLE

async def main():
    """
    Main function to manage BLE and Webots connections.
    It initializes BLE scanning, connects to Webots, and handles BLE data reception.
    """
    # Event to handle BLE disconnection
    disconnected_event = asyncio.Event()

    def disconnected_callback(client):
        """
        Callback function triggered when the BLE device disconnects.

        Args:
            client: The BLE client instance that disconnected.
        """
        print("Disconnected callback called!")
        disconnected_event.set()

    async def scan_ble_devices():
        """
        Scan for available BLE devices and print their details.
        """
        devices = await BleakScanner.discover()
        for d in devices:
            print(d)

    async def connect_to_webots():
        """
        Establish a TCP connection to the Webots simulation.
        """
        global webots_reader, tx_writer
        webots_reader, tx_writer = await asyncio.open_connection(WEBOTS_TCPIP_IP_ADDRESS, WEBOTS_TCPIP_PORT)
        print(f"Connected to Webots at {WEBOTS_TCPIP_IP_ADDRESS}:{WEBOTS_TCPIP_PORT}")

    def close_connection():
        """
        Close the TCP connection to Webots.
        """
        if tx_writer:
            tx_writer.close()
            print("Connection to Webots closed.")
        if webots_reader:
            webots_reader.close()
            print("Reader closed.")

    def cleanup():
        """
        Reset global variables and events for cleanup.
        """
        webots_reader, tx_writer = None, None
        disconnected_event = asyncio.Event()
        print("Cleanup completed.")

    async def test_webots_connection(message: str = "P"):
        """
        Send a test message to Webots to verify the connection.

        Args:
            message (str): The message to send to Webots. Defaults to "P".
        """
        if not tx_writer:
            print("Not connected to Webots.")
            return
        counter = 0
        forward = True
        while True:
            message = f"D,{counter}"
            tx_writer.write(message.encode('utf-8'))
            await tx_writer.drain()
            print(f"Sent: {message}")
            if forward:
                if counter > 10:
                    forward = False
                    counter -= 1
                counter += 1
            else:
                if counter < -10:
                    forward = True
                    counter += 1
                counter -= 1
            await asyncio.sleep(1)

    def decode_data(data: str) -> list[str] | None:
        """
        Decode the received data from the BLE device.

        Args:
            data (str): The data received from the BLE device.

        Returns:
            str: The decoded data.
        """
        # Glove states, 10 bits
        GLOVE_STATE_HW_INTERRUPT: Final[int] = 0b1000000000 
        GLOVE_STATE_FIST_CLOSED: Final[int] = 0b0100000000 
        GLOVE_STATE_NOT_IMPLEMENTED_4: Final[int] = 0b0010000000
        GLOVE_STATE_NOT_IMPLEMENTED_3: Final[int] = 0b0001000000
        GLOVE_STATE_NOT_IMPLEMENTED_2: Final[int] = 0b0000100000
        GLOVE_STATE_NOT_IMPLEMENTED_1: Final[int] = 0b0000010000
        GLOVE_STATE_RIGHT: Final[int] = 0b0000001000
        GLOVE_STATE_LEFT: Final[int] = 0b0000000100
        GLOVE_STATE_UP: Final[int] = 0b0000000010
        GLOVE_STATE_DOWN: Final[int] = 0b0000000001
        GLOVE_STATE_INITIAL: Final[int] = 0b0000000000

        # Webots command strings
        WEBOTS_CMD_FORMAT_STEERING_ANGLE: Final[str] = "A,{0}"
        WEBOTS_CMD_FORMAT_DRIVE: Final[str] = "D,{0}"
        WEBOTS_CMD_FORMAT_PARK: Final[str] = "P"

        WEBOTS_CMD_MAX_ANGLE: Final[int] = 30 # Maximum angle for Webots command
        WEBOTS_CMD_MAX_SPEED: Final[int] = 10 # Maximum angle for Webots command

        # create a variable called glove_state_bitmask and decode the string data to an 10bit integer
        glove_state_bitmask: int = int(data, 2)
        result_str_arr: list[str | None] = []

        # Map the glove state bitmask to Webots commands
        if (glove_state_bitmask & GLOVE_STATE_HW_INTERRUPT) or (glove_state_bitmask & GLOVE_STATE_FIST_CLOSED):
            result_str_arr.append(WEBOTS_CMD_FORMAT_PARK)
        else:
            if (glove_state_bitmask & GLOVE_STATE_DOWN) and (glove_state_bitmask & GLOVE_STATE_UP):
                result_str_arr.append(WEBOTS_CMD_FORMAT_DRIVE.format(0)) # Should not be possible, but just in case, default to 0
            elif glove_state_bitmask & GLOVE_STATE_DOWN:
                result_str_arr.append(WEBOTS_CMD_FORMAT_DRIVE.format(WEBOTS_CMD_MAX_SPEED))
            elif glove_state_bitmask & GLOVE_STATE_UP:
                result_str_arr.append(WEBOTS_CMD_FORMAT_DRIVE.format(-WEBOTS_CMD_MAX_SPEED))
            else:
                result_str_arr.append(WEBOTS_CMD_FORMAT_DRIVE.format(0))

            if (glove_state_bitmask & GLOVE_STATE_RIGHT) and (glove_state_bitmask & GLOVE_STATE_LEFT):
                result_str_arr.append(WEBOTS_CMD_FORMAT_STEERING_ANGLE.format(0)) # Should not be possible, but just in case, default to 0
            elif glove_state_bitmask & GLOVE_STATE_LEFT:
                result_str_arr.append(WEBOTS_CMD_FORMAT_STEERING_ANGLE.format(-WEBOTS_CMD_MAX_ANGLE))
            elif glove_state_bitmask & GLOVE_STATE_RIGHT:
                result_str_arr.append(WEBOTS_CMD_FORMAT_STEERING_ANGLE.format(WEBOTS_CMD_MAX_ANGLE))
            else:
                result_str_arr.append(WEBOTS_CMD_FORMAT_STEERING_ANGLE.format(0))

        results: list[str] = [element for element in result_str_arr if element is not None]

        # Return the decoded string or None if empty
        return results or None

    async def rx_from_ble():
        """
        Receive data from the BLE device and forward it to Webots.
        """
        async with BleakClient(GLOVE_ESP32_BLE_ADDRESS, disconnected_callback=disconnected_callback) as client:
            while True:
                data = await client.read_gatt_char(GLOVE_ESP32_BLE_CHARACTERISTIC_UUID)
                if data:
                    cmds_now = data.decode('utf-8')
                    global cmds_prev
                    if (cmds_now != cmds_prev):
                        print(f"Received: {cmds_now}")
                        
                        webots_cmds: list[str] | None = None
                        
                        try:
                            webots_cmds = decode_data(cmds_now)
                        except:
                            print("Failed to decode data.")
                        
                        if webots_cmds:
                            print(f"webots_cmds: {webots_cmds}")
                            await tx_to_webots(webots_cmds)
                        
                        cmds_prev = cmds_now

    async def tx_to_webots(messages: list[str]):
        """
        Send a message to Webots via the TCP connection.

        Args:
            message (str): The message to send.
        """
        for message in messages:
            tx_writer.write((message + ";").encode())  # Append a semi-colon delimiter to each message
            await tx_writer.drain()
            print(f"Sent: {message}")
    
    async def loop_iteration(use_scan=False, use_connect=True, use_test=False, use_rx=True):
        """
        Perform a single iteration of the main loop, handling BLE scanning, Webots connection,
        test message sending, and BLE data reception.

        Args:
            use_scan (bool): Whether to scan for BLE devices. Defaults to False.
            use_connect (bool): Whether to connect to Webots. Defaults to True.
            use_test (bool): Whether to send a test message to Webots. Defaults to False.
            use_rx (bool): Whether to receive data from BLE. Defaults to True.
        """
        is_scan_success = None
        is_connect_success = None
        is_test_success = None
        is_rx_success = None

        if use_scan:
            try:
                await scan_ble_devices()
                is_scan_success = True
            except:
                is_scan_success = False
                print("Failed to scan BLE devices.")
        
        if use_connect:
            try:
                if use_scan:
                    if is_scan_success:
                        await connect_to_webots()
                        is_connect_success = True
                    else:
                        is_connect_success = False
                else:
                    await connect_to_webots()
                    is_connect_success = True
            except:
                is_connect_success = False
                print("Failed to connect to Webots.")

        if use_test:
            try:
                if is_connect_success:
                    await test_webots_connection()
                    is_test_success = True
                else:
                    is_test_success = False
            except:
                is_test_success = False
                print("Failed to send test message to Webots.")
        if use_rx:
            try:
                if is_connect_success:
                    await rx_from_ble()
                    is_rx_success = True
                else:
                    is_rx_success = False
            except:
                is_rx_success = False
                print("Failed to receive data from BLE device.")

        cleanup()
    
    # Main loop to continuously run the BLE and Webots connection tasks
    while True:
        await loop_iteration()

if __name__ == "__main__":
    """
    Entry point for the script. Runs the main function using asyncio.
    """
    asyncio.run(main())
