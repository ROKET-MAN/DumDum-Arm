import asyncio
import sys
import os
from bleak import BleakClient
import time

# The MAC address from your bluetoothctl output
MINIARM_ADDRESS = "48:87:2D:68:1C:4F"

# The unique handles of the characteristics we want to test.
# Both have 'write' properties, so one of them should be the correct one for commands.
WRITE_CHAR_HANDLE_1 = 24
WRITE_CHAR_HANDLE_2 = 32

def format_command(servo_updates, move_time=500):
    """
    Formats a command string for the Hiwonder miniArm.
    Example: s#1P1500#2P1500#T1000\r\n
    :param servo_updates: A dictionary of {'joint_name': new_position}
    :param move_time: The duration of the movement in milliseconds.
    :return: A byte string of the formatted command.
    """
    command = "s"
    # Placeholder for servo IDs, you may need to confirm these.
    servo_ids = {
        'base': 1, 'shoulder': 2, 'elbow': 3,
        'wrist': 4, 'gripper': 5
    }
    for joint, position in servo_updates.items():
        servo_id = servo_ids.get(joint)
        if servo_id:
            command += f"#{servo_id}P{int(position)}"
    command += f"#T{int(move_time)}\r\n"
    return command.encode('utf-8')

async def send_test_commands(client):
    """Sends a series of test commands to different handles."""
    # Test our original string command format on both handles
    command_data_1 = format_command(servo_updates={'shoulder': 2000}, move_time=1000)
    
    print(f"Testing handle {WRITE_CHAR_HANDLE_1}...")
    try:
        await client.write_gatt_char(WRITE_CHAR_HANDLE_1, command_data_1)
        print(f"Command sent successfully to handle {WRITE_CHAR_HANDLE_1}. Waiting...")
        await asyncio.sleep(2)
    except Exception as e:
        print(f"Failed to write to handle {WRITE_CHAR_HANDLE_1}: {e}")

    print(f"\nTesting handle {WRITE_CHAR_HANDLE_2}...")
    try:
        await client.write_gatt_char(WRITE_CHAR_HANDLE_2, command_data_1)
        print(f"Command sent successfully to handle {WRITE_CHAR_HANDLE_2}. Waiting...")
        await asyncio.sleep(2)
    except Exception as e:
        print(f"Failed to write to handle {WRITE_CHAR_HANDLE_2}: {e}")
    
    # Try a simple, raw byte command as an alternative
    print("\nTesting a raw byte command on both handles...")
    raw_command = bytes([0xFF, 0x01, 0x00, 0x5A]) # Example raw command
    print(f"Testing handle {WRITE_CHAR_HANDLE_1} with raw command...")
    try:
        await client.write_gatt_char(WRITE_CHAR_HANDLE_1, raw_command)
        print(f"Raw command sent successfully to handle {WRITE_CHAR_HANDLE_1}. Waiting...")
        await asyncio.sleep(2)
    except Exception as e:
        print(f"Failed to write to handle {WRITE_CHAR_HANDLE_1}: {e}")

    print(f"Testing handle {WRITE_CHAR_HANDLE_2} with raw command...")
    try:
        await client.write_gatt_char(WRITE_CHAR_HANDLE_2, raw_command)
        print(f"Raw command sent successfully to handle {WRITE_CHAR_HANDLE_2}. Waiting...")
        await asyncio.sleep(2)
    except Exception as e:
        print(f"Failed to write to handle {WRITE_CHAR_HANDLE_2}: {e}")

async def main(address):
    print(f"Connecting to BLE device at {address}...")
    try:
        async with BleakClient(address) as client:
            print(f"Connected: {client.is_connected}")
            
            # Since the arm disconnects when you press the button, you must run this script
            # immediately after connecting the power but BEFORE pressing the green button.
            # We will try both command handles and some different command formats.
            
            await send_test_commands(client)
            
    except Exception as e:
        print(f"BLE connection failed: {e}")
        print("Possible reasons: Device is not powered on, out of range, or already connected.")

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Warning: This script must be run with sudo for BLE communication.")
        print("Please run with: sudo python3 -m src.hue.coms.bleak_test")
        sys.exit(1)
        
    asyncio.run(main(MINIARM_ADDRESS))