#!/usr/bin/env python3
"""
test_vision_msg.py - Test script to verify CUSTOM_VISION_DATA (msg 189) reception in SITL.

Usage:
    # First, start SITL:
    #   cd ~/ardupilot && sim_vehicle.py -v ArduCopter --console --map
    #
    # Then run this script:
    #   python3 test_vision_msg.py
    #
    # You should see in the SITL console:
    #   "VEST: First vision msg received! seq=0 flg=3"
    #   "VEST: seq=N rpe=0.5 flg=3 acc=1 hlth=1"

"""
import sys
import time
import math
import os

# Add mavlink generated headers to path
sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))

from pymavlink import mavutil

# --- Configuration ---
SITL_ADDRESS = "udp:127.0.0.1:14550"  # Default SITL MAVLink port
SEND_RATE_HZ = 80                      # Send rate (simulating ~30fps camera)
DURATION_SEC = 30                      # How long to send (seconds)

# --- Custom Vision Flags ---
FLAG_VALID_DETECTION = (1 << 0)
FLAG_NEW_FRAME       = (1 << 1)
FLAG_LOW_QUALITY     = (1 << 2)
FLAG_STALE_FRAME     = (1 << 3)

def main():
    print(f"Connecting to SITL at {SITL_ADDRESS}...")
    conn = mavutil.mavlink_connection(SITL_ADDRESS)
    
    # Wait for heartbeat to confirm connection
    print("Waiting for heartbeat...")
    conn.wait_heartbeat()
    print(f"Connected! Target system={conn.target_system}, component={conn.target_component}")
    
    # Check if CUSTOM_VISION_DATA is available in this mavlink dialect
    if not hasattr(conn.mav, 'custom_vision_data_send'):
        print("ERROR: custom_vision_data_send not found in pymavlink!")
        print("You need to regenerate pymavlink with the custom message.")
        print("Run:")
        print(f"  pip3 install --user ~/ardupilot/modules/mavlink/pymavlink")
        print("Or use mavgen to generate the library.")
        sys.exit(1)
    
    print(f"\n--- Sending CUSTOM_VISION_DATA at {SEND_RATE_HZ}Hz for {DURATION_SEC}s ---")
    print("Watch the SITL console for 'VEST:' messages\n")
    
    seq = 0
    t_start = time.time()
    dt = 1.0 / SEND_RATE_HZ
    
    # Simulate a marker at a fixed position in body frame:
    # 2m ahead, 0m right, 1m below the drone
    marker_x = 2.0   # forward
    marker_y = 0.0   # right
    marker_z = 1.0   # down
    
    try:
        while (time.time() - t_start) < DURATION_SEC:
            t_now = time.time()
            
            # Flags: valid detection + new frame
            flags = FLAG_VALID_DETECTION | FLAG_NEW_FRAME
            
            # Simulated reprojection error (low = good)
            reproj_error = 0.5  # pixels
            
            # Optional: add some sinusoidal motion to test velocity estimation
            t_elapsed = t_now - t_start
            offset_x = 0.1 * math.sin(2.0 * math.pi * 0.2 * t_elapsed)  # 0.2Hz, 0.1m amplitude
            
            # Send the message
            conn.mav.custom_vision_data_send(
                int(t_now * 1e6),                          # time_usec
                seq,                                        # seq
                [marker_x + offset_x, marker_y, marker_z], # position (body frame)
                reproj_error,                               # reprojection_error
                flags                                       # flags
            )
            
            if seq % SEND_RATE_HZ == 0:
                print(f"  Sent seq={seq}, pos=[{marker_x+offset_x:.3f}, {marker_y:.3f}, {marker_z:.3f}], "
                      f"rpe={reproj_error:.1f}, flags=0x{flags:02x}")
            
            seq += 1
            
            # Maintain send rate
            t_sleep = dt - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    print(f"\n--- Done! Sent {seq} messages in {time.time()-t_start:.1f}s ---")
    print(f"Effective rate: {seq/(time.time()-t_start):.1f} Hz")
    print("\nCheck SITL for:")
    print("  1. 'VEST: First vision msg received!' → handler is reached")
    print("  2. 'VEST: seq=N rpe=0.5 flg=3 acc=1 hlth=1' → estimator is working")
    print("  3. VRAW log in .bin file → raw reception data")
    print("  4. VEST log in .bin file → filtered estimator output")
    print("\nIf NO 'VEST:' messages appear:")
    print("  - The message is not reaching the handler")
    print("  - Check: is msg 189 ID correct in the generated headers?")
    print("  - Check: is MODE_LLC_ENABLED defined? (handler is inside #if MODE_LLC_ENABLED)")

if __name__ == "__main__":
    main()
