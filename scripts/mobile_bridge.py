"""AERO Mobile LAN Bridge.

Forwards external Wi-Fi / mobile traffic on Windows (e.g. http://10.208.106.188:8000 and :8080)
directly into WSL2 local dashboard (http://127.0.0.1:8000).
Supports full bidirectional streaming for HTTP, WebSockets, and MJPEG video.
"""
import socket
import threading
import time


def forward(src: socket.socket, dst: socket.socket) -> None:
    try:
        while True:
            data = src.recv(8192)
            if not data:
                break
            dst.sendall(data)
    except Exception:
        pass
    finally:
        try:
            src.close()
        except Exception:
            pass
        try:
            dst.close()
        except Exception:
            pass


def handle_client(client_sock: socket.socket, target_host: str, target_port: int) -> None:
    try:
        target_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        target_sock.connect((target_host, target_port))
        t1 = threading.Thread(target=forward, args=(client_sock, target_sock), daemon=True)
        t2 = threading.Thread(target=forward, args=(target_sock, client_sock), daemon=True)
        t1.start()
        t2.start()
    except Exception as e:
        try:
            client_sock.close()
        except Exception:
            pass


def start_server(listen_ip: str, listen_port: int, target_host: str = '127.0.0.1', target_port: int = 8000) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((listen_ip, listen_port))
    except Exception as e:
        print(f"[!] Could not bind {listen_ip}:{listen_port} - {e}")
        return

    server.listen(100)
    print(f"[OK] Mobile Bridge listening on http://{listen_ip}:{listen_port} -> forwarding to http://{target_host}:{target_port}")
    while True:
        try:
            client, addr = server.accept()
            threading.Thread(target=handle_client, args=(client, target_host, target_port), daemon=True).start()
        except Exception:
            break


def main():
    threads = []
    # Listen on Wi-Fi IP port 8000
    t1 = threading.Thread(target=start_server, args=('10.208.106.188', 8000), daemon=True)
    t1.start()
    threads.append(t1)

    # Also listen on 0.0.0.0:8080 as backup
    t2 = threading.Thread(target=start_server, args=('0.0.0.0', 8080), daemon=True)
    t2.start()
    threads.append(t2)

    for t in threads:
        t.join()


if __name__ == '__main__':
    main()
