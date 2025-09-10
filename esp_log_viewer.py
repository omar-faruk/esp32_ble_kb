#!/usr/bin/python3

import socket
import threading


if __name__=='__main__':
    HOST = "0.0.0.0"   # Listen on all interfaces
    PORT = 9999
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((HOST, PORT))
        print(f"[*] UDP listener started on {HOST}:{PORT}")

        while True:
            data, addr = s.recvfrom(4096)  # 4096-byte buffer
            try:
                text = data.decode(errors="ignore")
            except UnicodeDecodeError:
                text = str(data)
            print(f"[{addr}] {text}")

