import socket

HOST = "10.2.137.115"
PORT = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print(f"✅ Server listening on {HOST}:{PORT}")

while True:
    conn, addr = server_socket.accept()
    print(f"🔌 Connected by {addr}")
    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print("📩 Received:", data.decode().strip())