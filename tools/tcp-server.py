import socket
import subprocess
import threading

from pyngrok import ngrok

class TCPServer():
    tcp_host: str
    tcp_port: str
    sock: socket
    running: bool = True
    server_up: bool = False
    connection_string: str
    client_can_send: dict
    url: str
    port: str

    def __init__(
        self, 
        tcp_host,
        tcp_port,
    ) -> None:
        
        self.connection_string = ""
        self.client_can_send = {}
        
        output = subprocess.getoutput('lsof -t -i tcp:9999 | xargs kill -9')
        output = subprocess.getoutput('lsof -t -i tcp:9999 | xargs kill -9')
        output = subprocess.getoutput('lsof -t -i tcp:9999 | xargs kill -9')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
        # self.sock = socket.socket(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((tcp_host, tcp_port))
        self.sock.listen(1)

        output = subprocess.getoutput('kill -9 "$(pgrep ngrok)"')
        print("TCP", output)

        if (
            connection_string := ngrok.connect(str(tcp_port), "tcp").public_url
        ) is None:
            print("TCP", "ngrok not running")

        self.url, self.port = connection_string.strip("tcp://").split(":")
        print("tcp url: " + self.url)
        print("tcp port: " + self.port)

    def start(self) -> None:
        if self.server_up:
            return
        self.running = True
        thread = threading.Thread(target=self.tcp_callback)
        thread.start()
        self.server_up = True

    def tcp_callback(self):
        while self.running:
            try:
                # Wait for a connection
                print("TCP", "Waiting for a connection ...")
                connection, client_address = self.sock.accept()
                print("TCP", f"... connection established from {client_address}")
                self.client_can_send[client_address[1]] = True

                # Receive the message, send a response
                while True:
                    if self.client_can_send[client_address[1]] == True:
                        self.client_can_send[client_address[1]] = False
                        msg = "GSM TEST END"
                        msg_bytes = msg.encode(encoding="utf-8")
                        connection.sendall(msg_bytes)
                        # connection.close()
                        # break
                
                    if data := connection.recv(1024).strip():
                        print("TCP", "Received: " + str(data))
                        connection.close()
                        break
            except KeyboardInterrupt:
                if connection:
                    connection.close()
                self.sock.close()

    def close(self) -> None:
        self.server_up = False
        self.running = False
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        print("TCP", "Server closed.")
        ngrok.disconnect(self.connection_string)

TCP_HOST = "localhost"
TCP_PORT = 9999

# Example Usage
if __name__ == "__main__":
    server = TCPServer(TCP_HOST, TCP_PORT)
    server.start()