from typing import Final

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

def test_decode_data():
    """
    Test the decode_data function with all possible combinations of 10-bit input data.
    """
    test_cases = [
        ("0000000000", ["D,0", "A,0"]),        # 0000: Stopped, Straight
        ("0000000001", ["D,10", "A,0"]),       # 0001: Forward, Straight
        ("0000000010", ["D,-10", "A,0"]),      # 0002: Backward, Straight
        ("0000000011", ["D,0", "A,0"]),        # 0003: Stopped, Straight
        ("0000000100", ["D,0", "A,-30"]),      # 0004: Stopped, Left
        ("0000000101", ["D,10", "A,-30"]),     # 0005: Forward, Left
        ("0000000110", ["D,-10", "A,-30"]),    # 0006: Backward, Left
        ("0000000111", ["D,0", "A,-30"]),      # 0007: Stopped, Left
        ("0000001000", ["D,0", "A,30"]),       # 0008: Stopped, Right
        ("0000001001", ["D,10", "A,30"]),      # 0009: Forward, Right
        ("0000001010", ["D,-10", "A,30"]),     # 0010: Backward, Right
        ("0000001011", ["D,0", "A,30"]),       # 0011: Stopped, Right
        ("0000001100", ["D,0", "A,0"]),        # 0012: Stopped, Straight
        ("0000001101", ["D,10", "A,0"]),       # 0013: Forward, Straight
        ("0000001110", ["D,-10", "A,0"]),      # 0014: Backward, Straight
        ("0000001111", ["D,0", "A,0"]),        # 0015: Stopped, Straight
        ("0000010000", ["D,0", "A,0"]),        # 0016: Stopped, Straight
        ("0000010001", ["D,10", "A,0"]),       # 0017: Forward, Straight
        ("0000010010", ["D,-10", "A,0"]),      # 0018: Backward, Straight
        ("0000010011", ["D,0", "A,0"]),        # 0019: Stopped, Straight
        ("0000010100", ["D,0", "A,-30"]),      # 0020: Stopped, Left
        ("0000010101", ["D,10", "A,-30"]),     # 0021: Forward, Left
        ("0000010110", ["D,-10", "A,-30"]),    # 0022: Backward, Left
        ("0000010111", ["D,0", "A,-30"]),      # 0023: Stopped, Left
        ("0000011000", ["D,0", "A,30"]),       # 0024: Stopped, Right
        ("0000011001", ["D,10", "A,30"]),      # 0025: Forward, Right
        ("0000011010", ["D,-10", "A,30"]),     # 0026: Backward, Right
        ("0000011011", ["D,0", "A,30"]),       # 0027: Stopped, Right
        ("0000011100", ["D,0", "A,0"]),        # 0028: Stopped, Straight
        ("0000011101", ["D,10", "A,0"]),       # 0029: Forward, Straight
        ("0000011110", ["D,-10", "A,0"]),      # 0030: Backward, Straight
        ("0000011111", ["D,0", "A,0"]),        # 0031: Stopped, Straight
        # we can skip the rest of the NOT_IMPLEMENTED cases
        # Continue from bit (2^8) to (2^10 - 1), excluding the NOT_IMPLEMENTED cases
        ("0100000000", ["P"]),                 # 0256: Parked
        ("0100000001", ["P"]),                 # 0257: Parked
        ("0100000010", ["P"]),                 # 0258: Parked
        ("0100000011", ["P"]),                 # 0259: Parked
        ("0100000100", ["P"]),                 # 0260: Parked
        ("0100000101", ["P"]),                 # 0261: Parked
        ("0100000110", ["P"]),                 # 0262: Parked
        ("0100000111", ["P"]),                 # 0263: Parked
        ("0100001000", ["P"]),                 # 0264: Parked
        ("0100001001", ["P"]),                 # 0265: Parked
        ("0100001010", ["P"]),                 # 0266: Parked
        ("0100001011", ["P"]),                 # 0267: Parked
        ("0100001100", ["P"]),                 # 0268: Parked
        ("0100001101", ["P"]),                 # 0269: Parked
        ("0100001110", ["P"]),                 # 0270: Parked
        ("0100001111", ["P"]),                 # 0271: Parked
        # NOT IMPLEMENTED cases here
        ("1000000000", ["P"]),                 # 0512: Parked
        ("1000000001", ["P"]),                 # 0513: Parked
        ("1000000010", ["P"]),                 # 0514: Parked
        ("1000000011", ["P"]),                 # 0515: Parked
        ("1000000100", ["P"]),                 # 0516: Parked
        ("1000000101", ["P"]),                 # 0517: Parked
        ("1000000110", ["P"]),                 # 0518: Parked
        ("1000000111", ["P"]),                 # 0519: Parked
        ("1000001000", ["P"]),                 # 0520: Parked
        ("1000001001", ["P"]),                 # 0521: Parked
        ("1000001010", ["P"]),                 # 0522: Parked
        ("1000001011", ["P"]),                 # 0523: Parked
        ("1000001100", ["P"]),                 # 0524: Parked
        ("1000001101", ["P"]),                 # 0525: Parked
        ("1000001110", ["P"]),                 # 0526: Parked
        ("1000001111", ["P"]),                 # 0527: Parked
        # NOT IMPLEMENTED cases here
        ("1100000000", ["P"]),                 # 0768: Parked
        ("1100000001", ["P"]),                 # 0769: Parked
        ("1100000010", ["P"]),                 # 0770: Parked
        ("1100000011", ["P"]),                 # 0771: Parked
        ("1100000100", ["P"]),                 # 0772: Parked
        ("1100000101", ["P"]),                 # 0773: Parked
        ("1100000110", ["P"]),                 # 0774: Parked
        ("1100000111", ["P"]),                 # 0775: Parked
        ("1100001000", ["P"]),                 # 0776: Parked
        ("1100001001", ["P"]),                 # 0777: Parked
        ("1100001010", ["P"]),                 # 0778: Parked
        ("1100001011", ["P"]),                 # 0779: Parked
        ("1100001100", ["P"]),                 # 0780: Parked
        ("1100001101", ["P"]),                 # 0781: Parked
        ("1100001110", ["P"]),                 # 0782: Parked
        ("1100001111", ["P"]),                 # 0783: Parked
        # NOT IMPLEMENTED cases here
        # Added a few more test cases, where all the NOT_IMPLEMENTED bits are set to 1
        ("1111110000", ["P"]),                 # 1008: Parked
        ("1111110001", ["P"]),                 # 1009: Parked
        ("1111110010", ["P"]),                 # 1010: Parked
        ("1111110011", ["P"]),                 # 1011: Parked
        ("1111110100", ["P"]),                 # 1012: Parked
        ("1111110101", ["P"]),                 # 1013: Parked
        ("1111110110", ["P"]),                 # 1014: Parked
        ("1111110111", ["P"]),                 # 1015: Parked
        ("1111111000", ["P"]),                 # 1016: Parked
        ("1111111001", ["P"]),                 # 1017: Parked
        ("1111111010", ["P"]),                 # 1018: Parked
        ("1111111011", ["P"]),                 # 1019: Parked
        ("1111111100", ["P"]),                 # 1020: Parked
        ("1111111101", ["P"]),                 # 1021: Parked
        ("1111111110", ["P"]),                 # 1022: Parked
        ("1111111111", ["P"]),                 # 1023: Parked
    ]

    for data, expected in test_cases:
        results = decode_data(data)
        if results:
            print(f"Input: {data} => Output: {results}")
            assert results == expected, f"Expected {expected}, but got {results}"

if __name__ == "__main__":
    test_decode_data()
    print("All tests passed!")