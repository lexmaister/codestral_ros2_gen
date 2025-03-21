#!/usr/bin/env python3

import ipaddress
from typing import List
import re
import logging

from codestral_ros2_gen import logger_main

logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


def parse_network_targets(network_spec: str) -> List[str]:
    """
    Parse various network target specifications and return a list of host IP addresses.

    Supported formats:
        * Standard CIDR:         "192.168.1.0/24"
        * IP range:              "192.168.1.1-192.168.1.254"
        * Single IP:             "192.168.1.1"
        * Multiple networks:     "192.168.1.0/24,10.0.0.0/16"
        * Octet ranges:          "192.168.10-20.1-254"
        * Wildcard notation:     "192.168.1.*"

    Args:
        network_spec (str): String specifying network(s) to scan

    Returns:
        List[str]: List of IP addresses to scan

    Raises:
        ValueError: If the network specification is not in a recognized format.
        TypeError: If the network specification contains invalid types.
    """
    # Remove whitespace and split by commas for multiple networks
    networks = [n.strip() for n in network_spec.split(",")]
    result_hosts = set()

    for network in networks:
        # Try processing as standard CIDR notation
        if "/" in network and not any(c in network for c in ["*", "-"]):
            try:
                ip_network = ipaddress.ip_network(network, strict=False)
                if ip_network.version == 4:
                    # For IPv4, exclude network and broadcast addresses
                    result_hosts.update(str(ip) for ip in ip_network.hosts())
                else:
                    # For IPv6, include all addresses
                    result_hosts.update(str(ip) for ip in ip_network)
                continue
            except ValueError:
                # Not a valid CIDR, try other formats
                pass

        # Try processing as IP range (e.g., 192.168.1.1-192.168.1.254)
        if "-" in network and network.count(".") >= 6:  # At least 7 dots for a range
            try:
                start_ip, end_ip = network.split("-")
                start_ip = ipaddress.ip_address(start_ip.strip())
                end_ip = ipaddress.ip_address(end_ip.strip())

                # Generate all IPs in the range
                current = int(start_ip)
                end = int(end_ip)

                while current <= end:
                    result_hosts.add(str(ipaddress.ip_address(current)))
                    current += 1
                continue
            except (ValueError, TypeError):
                # Not a valid IP range, try other formats
                pass

        # Try processing as octet ranges (e.g., 192.168.10-20.1-254)
        if re.search(r"\d+-\d+", network):
            try:
                # Parse each octet which may contain ranges
                octets = network.split(".")
                if len(octets) != 4:
                    raise ValueError("Invalid IP format")

                # Generate all combinations
                octet_values = []
                for octet in octets:
                    if "-" in octet:
                        start, end = map(int, octet.split("-"))
                        octet_values.append(list(range(start, end + 1)))
                    else:
                        octet_values.append([int(octet)])

                # Generate all IP combinations
                for o1 in octet_values[0]:
                    for o2 in octet_values[1]:
                        for o3 in octet_values[2]:
                            for o4 in octet_values[3]:
                                result_hosts.add(f"{o1}.{o2}.{o3}.{o4}")
                continue
            except (ValueError, TypeError):
                # Not a valid octet range, try other formats
                pass

        # Try processing as wildcard notation (e.g., 192.168.1.*)
        if "*" in network:
            try:
                parts = network.split(".")
                if len(parts) != 4:
                    raise ValueError("Invalid IP format")

                # Generate all combinations
                octet_values = []
                for i, part in enumerate(parts):
                    if part == "*":
                        octet_values.append(list(range(0, 256)))
                    else:
                        octet_values.append([int(part)])

                # Generate all IP combinations
                for o1 in octet_values[0]:
                    for o2 in octet_values[1]:
                        for o3 in octet_values[2]:
                            for o4 in octet_values[3]:
                                result_hosts.add(f"{o1}.{o2}.{o3}.{o4}")
                continue
            except (ValueError, TypeError):
                # Not a valid wildcard notation, try other formats
                pass

        # Try as a single IP address
        try:
            ip = ipaddress.ip_address(network)
            result_hosts.add(str(ip))
            continue
        except ValueError:
            # Not a valid IP address
            pass

        # If we reach here, the format wasn't recognized
        logger.warning(f"Could not parse network format: '{network}'")

    return sorted(list(result_hosts))
