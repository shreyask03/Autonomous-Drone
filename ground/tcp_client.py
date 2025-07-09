import socket
import threading
import queue

class TCPClient:
    def __init__(self, host='192.168.1.53', port=8000):
        self.server_address = (host, port)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = None
        self.client_addr = None
        self.msg = None
        self.on = False
        self.send_lock = threading.Lock()
        self.ack_queue = queue.Queue() # thread-safe way to get ACK responses from Portenta after sending data
        self.msg_queue = queue.Queue() # thread-safe way to get general messages from Portenta
        self.connected_event = threading.Event()

    def connect(self):
        self.server_socket.bind(self.server_address)
        self.server_socket.listen(5)
        print(f"Server listening at {self.server_address[0]}:{self.server_address[1]}")
        self.on = True
        threading.Thread(target=self.recv_loop, daemon=True).start()

    def recv_loop(self):
        while self.on:
            try:
                self.client_socket, self.client_addr = self.server_socket.accept()
                print(f"Connection established with {self.client_addr}")
                self.connected_event.set()
                
                buffer = ""
                while self.on:
                    try:
                        data = self.client_socket.recv(1024).decode('utf-8')
                        if not data:
                            print("Client disconnected.")
                            break # break inner loop to close socket and allow reconnection
                        # print(f"Raw received chunk: {repr(data)}")
                        buffer += data
                        while '\n' in buffer:
                            line,buffer = buffer.split('\n',1)
                            line = line.strip()
                            # print(f"Line received: {line}")
                            if line == "ACK":
                                # print("Putting ACK in queue")
                                self.ack_queue.put("ACK")
                            elif line.startswith("TUNE"):
                                # print(f"Putting message in queue: {line}")
                                self.msg_queue.put(line)
                                # self.client_socket.send("ACK\n".encode('utf-8'))
                    except Exception as e:
                        print(f"Receive error in loop: {e}")
                        break # break to close and clean up
            except Exception as e:
                print(f"Accept error: {e}")   
            finally:
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
                    print("Cleaned up client socket, ready for reconnection.")
                    self.connected_event.clear()

    def disconnect(self):
        self.on = False
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        self.connected_event.clear()

    def send_data(self, msg):
        if not self.client_socket:
            print("Not connected to a client.")
            return

        with self.send_lock:
            try:
                print(f"Sending: {msg}")
                self.client_socket.send((msg + '\n').encode('utf-8'))

                # Wait specifically for an ACK
                while True:
                    try:
                        response = self.ack_queue.get(timeout=5).strip()
                        if response == "ACK":
                            print("ACK received.")
                            break
                        else:
                            print(f"Skipping unexpected message: {response}")
                    except queue.Empty:
                        print("ACK not received: Timeout occurred")
                        break
            except Exception as e:
                print(f"Transmit error: {e}")
    
    def get_msg(self):
        try:
            self.msg = self.msg_queue.get_nowait()
        except queue.Empty:
            self.msg =  None
    
    def parseData(self):
        if not self.msg: # if self.msg is None
            return 
        try:
            copy = self.msg
            l = len(self.msg)
            firstComma = copy.index(',')
            cmd = copy[:firstComma] # take command
            copy = copy[firstComma+1:] # edit msg
            if(cmd == "TUNE"):
                # do something
                gains = copy.split(',')
                parsed = {}
                for gain in gains:
                    if '=' in gain:
                        key,val = gain.split('=',1)
                        # print(f"{key}={val}")
                        parsed[key.strip()]=val.strip()
                # print(f"Successfully parsed {len(parsed)} pairs.")
                return parsed
                

            # elif(cmd == "POS"):
                # do something

        except Exception as e:
            print(f"Parsing error: {e}")
            return None
    
    def request_data(self):
        for attempt in range(2): # try up to 2 times
            # first send a request for data to the client with a command after to specify what data is needed
            self.send_data("REQUEST,TUNE")
            # wait for response on msg_queue
            try:
                resp = self.msg_queue.get(timeout=5)
                print(f"Received requested data.")
                self.msg = resp
                return self.parseData()
            except queue.Empty:
                print(f"Attempt {attempt + 1}: Did not receive requested data.")
        return None
