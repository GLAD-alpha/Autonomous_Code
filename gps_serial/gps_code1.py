import serial

# Open the serial port
ser = serial.Serial(
    port = '/dev/ttyUSB1',
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=8
)

# Check if the serial port is open
if ser.isOpen():
    print('Start receiving GPS data...')
else:
    print('Not working properly....')
    exit()

# Read and print NMEA sentences until the user enters 'q'
while True:
    #sentence = ser.readline().decode('ascii')
    #print(sentence)
    #data = ser.read(1000)
    #print(data)
    raw_sentence = ser.readline()
    try:
        sentence = raw_sentence.decode('ascii')
    except UnicodeDecodeError:

        print("Received Non-ASCII Data :", raw_sentence)

    op = input("Enter a command, or 'q' to quit: ")
    if op == 'q' or op == 'quit':
        print('Stopping serial process...')
        break

ser.close()
