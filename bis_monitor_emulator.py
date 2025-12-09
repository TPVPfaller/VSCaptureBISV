#!/usr/bin/env python3
"""
BIS Monitor Emulator for COM1
Emulates a BIS Vista monitor responding to VSCaptureBISV client requests.

This program receives serial commands on COM1 and responds with simulated BIS data.
Protocol:
- SPI Frame Header: 0xBAAB
- Frame format: [SPI_ID(2)] [DATA] [CRC(2)]
- Messages include processed data, spectral data, and raw EEG data
"""

import struct
import time
import random
import serial
import threading
import logging
from typing import Tuple, List

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Protocol Constants
SPI_ID = b'\xBA\xAB'
L1_DATA_PACKET = 0x01
L1_ACK_PACKET = 0x02
L1_NAK_PACKET = 0x03

M_DATA_RAW_EEG = 0x32
M_PROCESSED_VARS = 0x34
M_PROCESSED_VARS_AND_SPECTRA = 0x35

# Request message IDs
MSG_PROCESSED_DATA = 0x73
MSG_RAW_EEG = 0x6F
MSG_STOP_PROCESSED = 0x74
MSG_STOP_RAW_EEG = 0x70

class CRCCalculator:
    """Calculates CRC checksum for BIS protocol"""
    
    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """Compute 16-bit CRC checksum"""
        crc_sum = 0
        for byte in data:
            crc_sum += byte
        return crc_sum & 0xFFFF


class BISMonitorEmulator:
    """Emulates a BIS Vista monitor on serial port COM1"""
    
    def __init__(self, port: str = 'COM1', baudrate: int = 57600):
        """
        Initialize the BIS monitor emulator
        
        Args:
            port: Serial port name (e.g., 'COM1')
            baudrate: Serial port baud rate (default: 57600)
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.read_thread = None
        
        # Simulated device data
        self.seq_num = 0
        self.raw_eeg_counter = 0
        
    def open(self) -> bool:
        """Open serial port connection"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            logger.info(f"Opened serial port {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            return False
    
    def close(self):
        """Close serial port connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info(f"Closed serial port {self.port}")
    
    @staticmethod
    def create_frame(data: bytes) -> bytes:
        """
        Create a complete frame with SPI header and CRC checksum
        
        Frame format: [SPI_ID(2)] [DATA] [CRC(2)]
        """
        crc = CRCCalculator.compute_checksum(data)
        crc_bytes = struct.pack('<H', crc)
        return SPI_ID + data + crc_bytes
    
    def create_processed_data_response(self) -> bytes:
        """Create a processed variables data packet (M_PROCESSED_VARS)"""
        self.seq_num = (self.seq_num + 1) & 0xFFFF
        
        # Packet structure: [seq_id(2)][len(2)][type(2)][routing_id(4)][msg_id(4)][seq(2)][msg_len(2)][data...]
        
        # Simulated processed data
        bis_index = random.randint(30, 60)  # BIS index 0-100
        burst_suppress = random.randint(0, 500)  # BSR 0-1000 in 0.1% steps
        spectral_edge = random.randint(1000, 2000)  # in 0.01 Hz units
        emg = random.randint(0, 50)  # EMG dB
        signal_quality = random.randint(500, 1000)  # SQI 0-1000
        
        # Create message data (simplified - 120 bytes for processed_vars_msg)
        # Pack a fixed set of 20 integers, then pad to 120 bytes.
        vals = [
            0,      # dsc_id
            0,      # dsc_id_legal
            0,      # pic_id
            0,      # pic_id_legal
            2,      # dsc_numofchan
            0,      # quick_test_result
            0,      # filt_setting
            0,      # smoothing_setting
            0,      # spectral_art_mask
            0,      # bispectral_art_mask
            burst_suppress,
            spectral_edge,
            0,      # bis_bits
            bis_index,
            bis_index,
            bis_index,
            4000,   # total_power
            emg,    # emg_low
            signal_quality,
            0       # second_artifact
        ]

        message_data = struct.pack('<' + 'I' * len(vals), *vals)
        # Pad to 120 bytes
        if len(message_data) < 120:
            message_data = message_data.ljust(120, b'\x00')
        
        # Build packet
        routing_id = 0
        msg_id = M_PROCESSED_VARS
        msg_len = len(message_data)
        
        packet = struct.pack(
            '<HHHIIHHHH',
            self.seq_num,
            msg_len + 12,  # total data length
            L1_DATA_PACKET,
            routing_id,
            msg_id,
            self.seq_num,
            msg_len,
            0,
            0
        ) + message_data
        
        return self.create_frame(packet)
    
    def create_raw_eeg_response(self) -> bytes:
        """Create raw EEG data packet (M_DATA_RAW_EEG)"""
        self.seq_num = (self.seq_num + 1) & 0xFFFF
        self.raw_eeg_counter += 1
        
        # Raw EEG data: 2 channels, 4 bytes per sample (2 bytes per channel)
        num_channels = 2
        num_samples = 16  # Simulated EEG samples
        
        # Create simulated EEG data (16-bit values per channel)
        eeg_data = struct.pack('<HH', num_channels, 128)  # num_channels, sample_rate
        
        for i in range(num_samples):
            # Simulate EEG as sine wave
            ch1 = int(1000 * random.uniform(-1, 1))
            ch2 = int(1000 * random.uniform(-1, 1))
            eeg_data += struct.pack('<hh', ch1, ch2)
        
        msg_len = len(eeg_data)
        
        # Build packet
        routing_id = 0
        msg_id = M_DATA_RAW_EEG
        
        packet = struct.pack(
            '<HHHIIHHHH',
            self.seq_num,
            msg_len + 12,  # total data length
            L1_DATA_PACKET,
            routing_id,
            msg_id,
            self.seq_num,
            msg_len,
            0,
            0
        ) + eeg_data
        
        return self.create_frame(packet)
    
    def parse_request(self, data: bytes) -> Tuple[bool, str]:
        """
        Parse incoming request and identify the command
        
        Returns:
            Tuple of (success: bool, command_type: str)
        """
        if len(data) < 4:
            return False, "unknown"
        
        # Check SPI header
        if data[:2] != SPI_ID:
            return False, "invalid_header"
        
        # Extract and verify CRC
        if len(data) < 4:
            return False, "too_short"
        
        received_crc = struct.unpack('<H', data[-2:])[0]
        computed_crc = CRCCalculator.compute_checksum(data[2:-2])
        
        if received_crc != computed_crc:
            logger.warning(f"CRC mismatch: received={received_crc:04x}, computed={computed_crc:04x}")
            return False, "crc_error"
        
        # Parse the message
        try:
            payload = data[2:-2]  # Remove SPI header and CRC
            
            if len(payload) < 6:
                return False, "too_short"
            
            seq_id, data_len, packet_type = struct.unpack('<HHH', payload[:6])
            
            if packet_type != L1_DATA_PACKET:
                return False, "not_data_packet"
            
            if len(payload) < 12:
                return False, "incomplete_packet"
            
            routing_id, msg_id = struct.unpack('<II', payload[6:14])
            
            # Identify the command
            if msg_id == MSG_PROCESSED_DATA:
                return True, "processed_data"
            elif msg_id == MSG_RAW_EEG:
                return True, "raw_eeg"
            elif msg_id == MSG_STOP_PROCESSED:
                return True, "stop_processed"
            elif msg_id == MSG_STOP_RAW_EEG:
                return True, "stop_raw_eeg"
            else:
                return True, "unknown_msg"
                
        except Exception as e:
            logger.error(f"Error parsing request: {e}")
            return False, "parse_error"

    def _extract_frame_from_buffer(self, buffer: bytes, max_frame_len: int = 1024) -> (bytes, bytes):
        """Scan buffer for a complete frame by validating CRC.

        Returns a tuple (frame, remaining_buffer). If no complete frame found,
        returns (None, buffer).
        """
        start = 0
        while True:
            idx = buffer.find(SPI_ID, start)
            if idx == -1:
                return None, buffer

            # Try to find an end index where CRC matches
            # Minimum frame length: SPI(2) + minimal payload(1) + CRC(2) -> but we require at least 6 bytes of payload header
            min_total = idx + 2 + 6 + 2
            if len(buffer) < min_total:
                # not enough data yet
                return None, buffer

            # Search for candidate frame ends up to max_frame_len or available buffer
            max_search_end = min(len(buffer), idx + max_frame_len)

            found = False
            for end in range(min_total, max_search_end + 1):
                # last two bytes before end are CRC
                candidate = buffer[idx:end]
                if len(candidate) < 4:
                    continue
                received_crc = struct.unpack('<H', candidate[-2:])[0]
                payload = candidate[2:-2]
                computed_crc = CRCCalculator.compute_checksum(payload)
                if received_crc == computed_crc:
                    # Found a valid frame
                    remaining = buffer[end:]
                    return candidate, remaining

            # advance past this SPI_ID and keep searching
            start = idx + 1
    
    def handle_request(self, data: bytes) -> bytes:
        """Handle incoming request and generate appropriate response"""
        success, cmd_type = self.parse_request(data)
        
        if not success:
            logger.warning(f"Invalid request: {cmd_type}")
            return b""
        
        logger.info(f"Received command: {cmd_type}")
        
        # Generate response based on command
        if cmd_type == "processed_data":
            return self.create_processed_data_response()
        elif cmd_type == "raw_eeg":
            return self.create_raw_eeg_response()
        elif cmd_type in ["stop_processed", "stop_raw_eeg"]:
            logger.info(f"Stopping {cmd_type}")
            return b""
        else:
            logger.warning(f"Unknown command: {cmd_type}")
            return b""
    
    def read_loop(self):
        """Main read loop for processing incoming serial data"""
        buffer = b""
        frame_start_marker = b"\x00\x00"  # Common frame delimiter
        
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data
                    
                    # Extract complete frames by CRC validation
                    while True:
                        result = self._extract_frame_from_buffer(buffer)
                        if result is None:
                            break
                        frame, buffer = result
                        if frame is None:
                            break

                        logger.debug(f"Received frame: {frame.hex()}")
                        response = self.handle_request(frame)
                        if response:
                            try:
                                self.ser.write(response)
                                logger.debug(f"Sent response: {response.hex()}")
                            except Exception as e:
                                logger.error(f"Failed to write response: {e}")
                
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                break
    
    def start(self):
        """Start the emulator"""
        if not self.open():
            return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        logger.info("BIS Monitor Emulator started")
        return True
    
    def stop(self):
        """Stop the emulator"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        self.close()
        logger.info("BIS Monitor Emulator stopped")


def main():
    """Main entry point"""
    import sys
    
    port = 'COM1'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    emulator = BISMonitorEmulator(port=port)
    
    if not emulator.start():
        logger.error("Failed to start emulator")
        return 1
    
    logger.info(f"BIS Monitor Emulator running on {port}")
    logger.info("Press Ctrl+C to stop")
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        emulator.stop()
    
    return 0


if __name__ == '__main__':
    exit(main())
