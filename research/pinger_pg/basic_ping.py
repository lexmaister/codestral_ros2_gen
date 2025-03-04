#!/usr/bin/env python3

import socket
import struct
import os
import time
import select

# Global counter for ICMP packet identification
icmp_counter = 0


def ping(ip_addr, timeout=1, count=1):
    """
    Send ICMP echo request to a host and check for response.

    Args:
        ip_addr (str): IP address to ping
        timeout (float): Timeout in seconds to wait for response
        count (int): Number of ping attempts

    Returns:
        bool: True if host responds, False otherwise
    """
    # Check for root/admin privileges
    if os.name != "nt" and os.geteuid() != 0:
        raise PermissionError(
            "Raw socket operations require root privileges. Run with sudo."
        )

    # Create socket
    icmp_socket = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    icmp_socket.settimeout(timeout)

    # Use global counter for request identification
    global icmp_counter

    for seq in range(1, count + 1):
        try:
            # Increment counter and keep within 16 bits
            icmp_counter = (icmp_counter + 1) & 0xFFFF

            # Create ICMP packet using the counter instead of process_id
            packet = create_icmp_packet(icmp_counter, seq)

            # Send packet
            icmp_socket.sendto(packet, (ip_addr, 0))

            # Store current counter value to validate response
            current_id = icmp_counter

            # Wait for response
            start_time = time.time()

            while True:
                # Check if we've exceeded timeout
                if time.time() - start_time > timeout:
                    break

                # Use select to wait for data with timeout
                ready = select.select(
                    [icmp_socket], [], [], timeout - (time.time() - start_time)
                )
                if not ready[0]:  # Timeout
                    break

                # Receive packet
                recv_packet, addr = icmp_socket.recvfrom(1024)

                # Validate response with our counter value
                if validate_icmp_response(recv_packet, current_id):
                    icmp_socket.close()
                    return True

        except Exception as e:
            print(f"Error pinging {ip_addr}: {e}")

    icmp_socket.close()
    return False


def create_icmp_packet(identifier, seq=1):
    """Create an ICMP echo request packet"""
    # ICMP Echo Request (type=8, code=0)
    header = struct.pack("!BBHHH", 8, 0, 0, identifier, seq)
    data = b"abcdefghijklmnopqrstuvwxyz"

    # Calculate checksum on the header + data
    checksum = calculate_checksum(header + data)

    # Create header with calculated checksum
    header = struct.pack("!BBHHH", 8, 0, checksum, identifier, seq)

    return header + data


def calculate_checksum(data):
    """Calculate the checksum for the ICMP header"""
    checksum = 0
    count_to = (len(data) // 2) * 2

    for count in range(0, count_to, 2):
        this_val = data[count + 1] * 256 + data[count]
        checksum += this_val
        checksum &= 0xFFFFFFFF

    if count_to < len(data):
        checksum += data[len(data) - 1]
        checksum &= 0xFFFFFFFF

    checksum = (checksum >> 16) + (checksum & 0xFFFF)
    checksum += checksum >> 16
    answer = ~checksum
    answer &= 0xFFFF

    # Swap bytes
    answer = answer >> 8 | (answer << 8 & 0xFF00)
    return answer


def validate_icmp_response(recv_packet, identifier):
    """Validate that the received packet is an ICMP echo reply for our request"""
    if len(recv_packet) < 28:  # IP header (20) + ICMP header (8)
        return False

    icmp_header = recv_packet[20:28]
    type, code, _, packet_id, _ = struct.unpack("!BBHHH", icmp_header)

    # Type 0 is echo reply, and we verify it's our reply by checking identifier
    return type == 0 and code == 0 and packet_id == identifier


# Example usage
if __name__ == "__main__":
    import sys

    target_ip = "8.8.8.8"  # Google's DNS server

    if len(sys.argv) > 1:
        target_ip = sys.argv[1]

    start_time = time.time()
    if ping(target_ip, timeout=0.5):
        print(f"[+] Host {target_ip} is UP")
    else:
        print(f"[-] Host {target_ip} is DOWN")

    print(f"[*] Time elapsed: {time.time() - start_time:.3f}s")
