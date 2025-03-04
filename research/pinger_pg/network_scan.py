#!/usr/bin/env python3

import socket
import struct
import os
import time
import select
import ipaddress
import pandas as pd
from typing import List, Dict, Union, Any
import sys


class NetworkPinger:
    def __init__(self, timeout: float = 1, count: int = 1, verbose: bool = False):
        """
        Initialize the network pinger.

        Args:
            timeout (float): Timeout in seconds to wait for response
            count (int): Number of ping attempts per host
            verbose (bool): Whether to print progress information
        """
        self.timeout = timeout
        self.count = count
        self.verbose = verbose
        self.icmp_counter = 0
        self.scan_table = None

        # Check for root/admin privileges
        if os.name != "nt" and os.geteuid() != 0:
            raise PermissionError(
                "Raw socket operations require root privileges. Run with sudo."
            )

    def scan_network(self, network: str) -> pd.DataFrame:
        """
        Scan an entire network using a pandas DataFrame lookup table.

        Args:
            network (str): Network in CIDR notation (e.g., '192.168.1.0/24')

        Returns:
            pd.DataFrame: Scan results table with host status and timing info
        """
        try:
            # Parse the network
            ip_network = ipaddress.ip_network(network, strict=False)

            # Get list of hosts to scan (excluding network and broadcast for IPv4)
            hosts_to_scan = (
                [str(ip) for ip in ip_network.hosts()]
                if ip_network.version == 4
                else [str(ip) for ip in ip_network]
            )

            # Initialize lookup table
            self.scan_table = pd.DataFrame(
                {
                    "host": hosts_to_scan,
                    "counter": 0,
                    "is_up": False,
                    "response_time": None,
                    "attempts": 0,
                }
            )

            # Create a raw socket once for all pings
            icmp_socket = socket.socket(
                socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP
            )
            icmp_socket.settimeout(self.timeout)

            total_hosts = len(hosts_to_scan)

            try:
                # Process each host
                for index, host in enumerate(hosts_to_scan):
                    if self.verbose:
                        print(f"[*] Scanning {host} ({index+1}/{total_hosts})")

                    # Track start time
                    start_time = time.time()

                    # Perform ping
                    is_up = self._ping_host_with_table(icmp_socket, host, index)

                    # Calculate response time
                    response_time = time.time() - start_time if is_up else None

                    # Update results in table
                    self.scan_table.at[index, "is_up"] = is_up
                    self.scan_table.at[index, "response_time"] = response_time

                    if self.verbose:
                        status = "UP" if is_up else "DOWN"
                        print(f"[{'+'if is_up else '-'}] Host {host} is {status}")

            finally:
                # Always close the socket
                icmp_socket.close()

            return self.scan_table

        except ValueError as e:
            print(f"Invalid network format: {e}")
            return pd.DataFrame()

    def _ping_host_with_table(
        self, icmp_socket: socket.socket, host: str, index: int
    ) -> bool:
        """Internal method to ping a host using lookup table for tracking"""
        for attempt in range(self.count):
            try:
                # Increment counter and keep within 16 bits
                self.icmp_counter = (self.icmp_counter + 1) & 0xFFFF

                # Store counter value in table
                self.scan_table.at[index, "counter"] = self.icmp_counter
                self.scan_table.at[index, "attempts"] += 1

                # Create and send packet
                packet = self._create_icmp_packet(self.icmp_counter)
                icmp_socket.sendto(packet, (host, 0))

                # Wait for response
                start_time = time.time()

                while time.time() - start_time < self.timeout:
                    try:
                        # Use select to wait for data with timeout
                        ready = select.select(
                            [icmp_socket],
                            [],
                            [],
                            self.timeout - (time.time() - start_time),
                        )
                        if not ready[0]:  # Timeout
                            break

                        # Receive packet
                        recv_packet, addr = icmp_socket.recvfrom(1024)

                        # Get current counter from table
                        current_counter = self.scan_table.at[index, "counter"]

                        # Validate response
                        if self._validate_icmp_response(recv_packet, current_counter):
                            return True

                    except BlockingIOError:
                        # No data available, try again
                        continue

            except Exception as e:
                if self.verbose:
                    print(f"Error pinging {host}: {e}")

        return False

    def _create_icmp_packet(self, identifier: int, seq: int = 1) -> bytes:
        """Create an ICMP echo request packet"""
        # ICMP Echo Request (type=8, code=0)
        header = struct.pack("!BBHHH", 8, 0, 0, identifier, seq)
        data = b"abcdefghijklmnopqrstuvwxyz"

        # Calculate checksum on the header + data
        checksum = self._calculate_checksum(header + data)

        # Create header with calculated checksum
        header = struct.pack("!BBHHH", 8, 0, checksum, identifier, seq)

        return header + data

    def _calculate_checksum(self, data: bytes) -> int:
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

    def _validate_icmp_response(self, recv_packet: bytes, identifier: int) -> bool:
        """Validate that the received packet is an ICMP echo reply for our request"""
        if len(recv_packet) < 28:  # IP header (20) + ICMP header (8)
            return False

        icmp_header = recv_packet[20:28]
        type, code, _, packet_id, _ = struct.unpack("!BBHHH", icmp_header)

        # Type 0 is echo reply, and we verify it's our reply by checking identifier
        return type == 0 and code == 0 and packet_id == identifier

    def get_alive_hosts(self) -> List[str]:
        """Extract list of alive hosts from scan results"""
        if self.scan_table is None:
            return []

        return self.scan_table[self.scan_table["is_up"]]["host"].tolist()

    def get_statistics(self) -> Dict[str, Any]:
        """Get scan statistics"""
        if self.scan_table is None:
            return {}

        alive_count = self.scan_table["is_up"].sum()
        total_count = len(self.scan_table)
        avg_response = self.scan_table[self.scan_table["is_up"]]["response_time"].mean()

        return {
            "total_hosts": total_count,
            "alive_hosts": alive_count,
            "percent_alive": (
                (alive_count / total_count) * 100 if total_count > 0 else 0
            ),
            "avg_response_time": avg_response,
        }

    def export_to_csv(self, filename: str) -> bool:
        """Export scan results to CSV file"""
        if self.scan_table is None:
            return False

        try:
            self.scan_table.to_csv(filename, index=False)
            return True
        except Exception as e:
            print(f"Error exporting to CSV: {e}")
            return False


# Example usage
if __name__ == "__main__":
    if len(sys.argv) > 1:
        network = sys.argv[1]
    else:
        network = "192.168.1.0/24"  # Default network to scan

    try:
        pinger = NetworkPinger(timeout=0.5, count=1, verbose=True)
        print(f"[*] Scanning network: {network}")

        start_time = time.time()
        results = pinger.scan_network(network)
        elapsed = time.time() - start_time

        stats = pinger.get_statistics()
        alive_hosts = pinger.get_alive_hosts()

        print("\n--- Scan Summary ---")
        print(f"Network scanned: {network}")
        print(f"Total hosts: {stats['total_hosts']}")
        print(f"Alive hosts: {stats['alive_hosts']} ({stats['percent_alive']:.1f}%)")
        if stats.get("avg_response_time"):
            print(f"Average response time: {stats['avg_response_time']*1000:.2f} ms")
        print(f"Time elapsed: {elapsed:.2f} seconds")

        if alive_hosts:
            print("\nAlive hosts:")
            for host in alive_hosts:
                print(f"  {host}")

        # Export results to CSV
        # csv_file = f"scan_results_{int(time.time())}.csv"
        # if pinger.export_to_csv(csv_file):
        #     print(f"\nResults exported to {csv_file}")

    except PermissionError as e:
        print(e)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[!] Scan interrupted by user")
        sys.exit(0)
