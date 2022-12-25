import socket
import threading

HEADER = 1
PORT = 23
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = bytearray()
DISCONNECT_MESSAGE.append(ord('!'))
DISCONNECT_MESSAGE.append(ord('D'))
DISCONNECT_MESSAGE.append(ord('I'))
DISCONNECT_MESSAGE.append(ord('S'))

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)


def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    match = bytearray()
    match.append(ord('$'))
    match.append(ord('M'))
    match.append(ord('<'))
    while connected:
        temp = conn.recv(3)
        if (temp == match):
            # print(temp)
            msg_length = conn.recv(1)

            command = (int.from_bytes(conn.recv(1)))
            if msg_length:
                msg_length = int.from_bytes(msg_length)
                # print(msg_length)
                msg = conn.recv(msg_length)
                if msg == DISCONNECT_MESSAGE:
                    connected = False

                print(f"[{addr}] command={command} len={msg_length} msg={msg}")
                # conn.send("Msg received".encode(FORMAT))
                conn.recv(1)
            if (command == 109):
                data=bytearray()
                data.append(ord('$'))
                data.append(ord('M'))
                data.append(ord('>'))
                data.append((6))
                data.append((109))
                for i in range(0,6):
                    data.append((0))
                data.append((1))
                print(f'[SENDING] {data}')
                conn.send(data)

    conn.close()


def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}")


print("[STARTING] server is starting...")
start()
