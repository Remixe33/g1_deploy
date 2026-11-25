import ipaddress
import shutil
import subprocess


def print_web_addresses(label, server_address):
    host, port = server_address[:2]
    print(f"{label}: listening on {host}:{port}", flush=True)
    if host != "0.0.0.0":
        print(f"Open: http://{host}:{port}/", flush=True)
        return

    ip = shutil.which("ip")
    addresses = []
    if ip:
        result = subprocess.run(
            [ip, "-o", "-4", "address", "show", "up", "scope", "global"],
            capture_output=True, text=True, check=False)
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 4 and fields[2] == "inet":
                addresses.append((fields[1], ipaddress.IPv4Interface(fields[3]).ip))
    print(f"Open on this machine: http://127.0.0.1:{port}/", flush=True)
    for interface, address in dict.fromkeys(addresses):
        print(f"Open if reachable ({interface}): http://{address}:{port}/", flush=True)
    if not addresses:
        print("No LAN address detected. Use ./g1 interfaces to find a reachable PC2 IP.", flush=True)
