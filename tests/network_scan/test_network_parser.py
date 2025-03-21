#!/usr/bin/env python3

from codestral_ros2_gen.network_scan.network_parser import parse_network_targets


def test_cidr_notation():
    """Test standard CIDR notation parsing"""
    # Small network
    hosts = parse_network_targets("192.168.1.0/30")
    assert len(hosts) == 2  # /30 gives 2 usable IPs
    assert "192.168.1.1" in hosts
    assert "192.168.1.2" in hosts

    # Class C network
    hosts = parse_network_targets("192.168.1.0/24")
    assert len(hosts) == 254  # 256 minus network and broadcast
    assert "192.168.1.1" in hosts
    assert "192.168.1.254" in hosts
    assert "192.168.1.0" not in hosts  # Network address
    assert "192.168.1.255" not in hosts  # Broadcast address

    # IPv6 basic test (just checking if it parses)
    hosts = parse_network_targets("2001:db8::/126")
    assert len(hosts) == 4  # /126 gives 4 IPv6 addresses


def test_ip_range():
    """Test IP range notation"""
    # Small range
    hosts = parse_network_targets("192.168.1.5-192.168.1.10")
    assert len(hosts) == 6  # 6 IPs in this range
    assert "192.168.1.5" in hosts
    assert "192.168.1.7" in hosts
    assert "192.168.1.10" in hosts
    assert "192.168.1.4" not in hosts
    assert "192.168.1.11" not in hosts

    # Range crossing subnet
    hosts = parse_network_targets("192.168.1.250-192.168.2.5")
    assert len(hosts) == 12  # 250-255, 0-5 = 12 IPs
    assert "192.168.1.250" in hosts
    assert "192.168.1.255" in hosts
    assert "192.168.2.0" in hosts
    assert "192.168.2.5" in hosts


def test_single_ip():
    """Test single IP address"""
    hosts = parse_network_targets("192.168.1.1")
    assert len(hosts) == 1
    assert hosts[0] == "192.168.1.1"

    # IPv6 address
    hosts = parse_network_targets("2001:db8::1")
    assert len(hosts) == 1
    assert hosts[0] == "2001:db8::1"


def test_multiple_networks():
    """Test comma-separated network specifications"""
    # Multiple CIDRs
    hosts = parse_network_targets("192.168.1.0/30,10.0.0.0/30")
    assert len(hosts) == 4  # 2 from each /30
    assert "192.168.1.1" in hosts
    assert "192.168.1.2" in hosts
    assert "10.0.0.1" in hosts
    assert "10.0.0.2" in hosts

    # Mixed formats
    hosts = parse_network_targets("192.168.1.1,192.168.1.5-192.168.1.7,192.168.2.0/30")
    assert len(hosts) == 6  # 1 + 3 + 2
    assert "192.168.1.1" in hosts
    assert "192.168.1.5" in hosts
    assert "192.168.1.7" in hosts
    assert "192.168.2.1" in hosts
    assert "192.168.2.2" in hosts


def test_octet_ranges():
    """Test octet range notation"""
    # Single octet range
    hosts = parse_network_targets("192.168.1.5-10")
    assert len(hosts) == 6
    assert "192.168.1.5" in hosts
    assert "192.168.1.10" in hosts

    # Multiple octet ranges
    hosts = parse_network_targets("192.168.1-3.1-3")
    expected_count = 3 * 3  # 9 combinations
    assert len(hosts) == expected_count
    assert "192.168.1.1" in hosts
    assert "192.168.2.2" in hosts
    assert "192.168.3.3" in hosts

    # Complex example from the original question
    hosts = parse_network_targets("192.168.10-21.0-24")
    expected_count = 12 * 25  # 12 subnets, 25 hosts each
    assert len(hosts) == expected_count
    assert "192.168.10.0" in hosts
    assert "192.168.15.12" in hosts
    assert "192.168.21.24" in hosts


def test_wildcard_notation():
    """Test wildcard notation"""
    # Last octet wildcard
    hosts = parse_network_targets("192.168.1.*")
    assert len(hosts) == 256
    assert "192.168.1.0" in hosts
    assert "192.168.1.128" in hosts
    assert "192.168.1.255" in hosts

    # Second octet wildcard
    hosts = parse_network_targets("192.*.1.1")
    assert len(hosts) == 256
    assert "192.0.1.1" in hosts
    assert "192.128.1.1" in hosts
    assert "192.255.1.1" in hosts

    # Multiple wildcards (limited test due to size)
    hosts = parse_network_targets("192.168.*.*")
    assert len(hosts) == 256 * 256
    assert "192.168.0.0" in hosts
    assert "192.168.128.128" in hosts
    assert "192.168.255.255" in hosts


def test_edge_cases():
    """Test edge cases and error handling"""
    # Empty input
    hosts = parse_network_targets("")
    assert len(hosts) == 0

    # Whitespace handling
    hosts = parse_network_targets(" 192.168.1.1, 192.168.1.2 ")
    assert len(hosts) == 2
    assert "192.168.1.1" in hosts
    assert "192.168.1.2" in hosts

    # Invalid format
    hosts = parse_network_targets("invalid_format")
    assert len(hosts) == 0

    # Invalid IP in range
    hosts = parse_network_targets("192.168.1.1-192.168.1.300")
    assert len(hosts) == 0

    # Invalid CIDR
    hosts = parse_network_targets("192.168.1.0/33")
    assert len(hosts) == 0

    # Mixed valid and invalid
    hosts = parse_network_targets("192.168.1.1,invalid,192.168.1.3")
    assert len(hosts) == 2
    assert "192.168.1.1" in hosts
    assert "192.168.1.3" in hosts


def test_specific_cases():
    """Test specific cases mentioned in the requirements"""
    # Case 1: Use proper range notation instead of mixed CIDR and range
    # Instead of: "192.168.0/24.0/24"
    hosts = parse_network_targets("192.168.0-24.0-24")
    # 25 * 25 = 625 hosts
    assert len(hosts) == 625
    assert "192.168.0.0" in hosts
    assert "192.168.12.12" in hosts
    assert "192.168.24.24" in hosts

    # Case 2: Use proper range notation instead of mixed CIDR and range
    # Instead of: "192.168.10-21.0/24"
    hosts = parse_network_targets("192.168.10-21.0-24")
    # 12 * 25 = 300 hosts
    assert len(hosts) == 300
    assert "192.168.10.0" in hosts
    assert "192.168.15.12" in hosts
    assert "192.168.21.24" in hosts
