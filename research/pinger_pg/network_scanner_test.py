#!/home/alex/Projects/ros2/.venv/bin/python3

from codestral_ros2_gen.network_scanner.network_scanner import NetworkScanner

# Basic scan with results display
scanner = NetworkScanner()
hosts = scanner.scan("192.168.10.0/24")
print(scanner.format_results(hosts, show_all=False))
