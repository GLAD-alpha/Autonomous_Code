import serial

uart = []

def work(): 
    global uart
    global uart_split
    try: 
        recvpacket = sr.readline().decode('ascii')
        uart_split = recvpacket.split(",")
        if uart_split[0] == '$':
            print(uart_split[3], "N", uart_split[5], "E")
    except Exception as e:
        print("NOT CONNECTED:", e)

        try: 
            recvpacket = sr.readline().decode('utf-8')
        except UnicodeDecodeError:
            print("Unknown encoding")

try:
    sr = serial.Serial("/dev/ttyUSB1", 115200)
    if sr == True:
        print("Start Collecting GPS data.......")
except:
    print("Port Not Found")
    exit()
              
while True:
    uart = ""
    uart_split = []
    work()
    