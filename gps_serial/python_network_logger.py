import platform
import subprocess
import socket
from datetime import datetime

# Edit this config as needed
config = {
  "remote_host": "192.168.74.11",
  "remote_port": 3003,
  "output_filename_prefix": "Xavier gps data",
  "output_filename_extension": "gps"
}


# Novatel Pwrpak7 에 명령어 전달 
receiver_commands = (

    "UNLOGALL,",
    #데이터 타입 설정가능
    #LOG [데이터 타입] [받는 주기]
    
    "LOG GPGGA ONTIME 5",
    "LOG BESTPOS ONTIME 5",
    
    
    #"LOG RXSTATUSB ONCHANGED",
    #"LOG RAWEPHEMB ONNEW",
    #"LOG GLORAWEPHEMB ONNEW",
    #"LOG BESTPOSB ONTIME 1",
    #"LOG RANGEB ONTIME 1",
    #"LOG RXCONFIGA ONCE",
    #"LOG VERSIONA ONCE",
    #"LOG LOGLISTA ONCE",
    #"LOG PORTSTATSA ONTIME 10",
    #"LOG PROFILEINFOA ONCE",
    #"LOG HWMONITORA ONTIME 10"
)

def verify_receiver_is_reachable(remote_ip_string):
    # Option for the number of packets. argument differs by OS.
    param = '-n' if platform.system().lower()=='windows' else '-c'

    command = ['ping', param, '1', remote_ip_string]

    if subprocess.call(command, stdout=subprocess.DEVNULL) != 0:
        print("Error: Cannot ping host '{}', please check that it is reachable".format(remote_ip_string))
        exit(-1)
    else:
        print("Verified can ping '{}'".format(remote_ip_string))

def configure_receiver(host, port, commands):
    
    # TCP/IP
    #client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #client_socket.connect((host, port))

    # UDP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    client_socket.bind((host, port))
    
    receiver_replies_during_config = []

    # Send each command to receiver. Verify receiver replies "OK" for each.
    for command in commands:
        print("Sending: {}".format(command))
        command_accepted = False
        send_attempt_count = 0

        while not command_accepted:
            send_attempt_count += 1
            command = "{}\n".format(command)
            client_socket.send(command.encode("utf-8"))
            
            receiver_reply = client_socket.recv(4096)
            receiver_reply_string = str(receiver_reply.decode("utf-8"))
            receiver_replies_during_config.append(receiver_reply_string)

            if ("OK" in receiver_reply_string):
                command_accepted = True

            if send_attempt_count > 100:
                print("ERROR: Failed to get receiver ({}:{}) to accept command: {}".format(host, port, command))
                exit(-1)
    return client_socket, receiver_replies_during_config

def log_data(client_socket, output_filename_prefix, output_filename_extension, receiver_replies_during_config):
    # Define output filename with prefix + date (YYYYMMDDHHMM) + extension
    date_time = datetime.today().strftime('%Y%m%d%H%M')
    output_filename = "{}-{}.{}".format(output_filename_prefix, date_time, output_filename_extension)

    try:
        with open(output_filename, 'w') as output_file:
            # First, write out receiver replies during config time to log file
            for reply in receiver_replies_during_config:
                output_file.write(reply)

            # Now, continually get data from receiver and write to the log
            print("\nBeginning logging to file \"{}\". Terminate logging with CTRL+C.\n\n".format(output_filename))
            while True:
                new_receiver_data = client_socket.recv(4096)
                new_receiver_string = str(new_receiver_data.decode('utf-8')) 
                print(new_receiver_string)

                # Optionally, could add logic here to compute the checksum for the log and decide if the received log is valid
                # [Put checksum code here]

                # Write the log from the receiver to file
                output_file.write(new_receiver_string)
    except:
        output_file.close()

    return output_filename

def received_data_preprocess(raw_gps_data):
    
    ck_list = []

    print("Checking received data.......")

    ck_list.append(raw_gps_data)

    '''
    1. What type data Pwrpak7 receive?
    2. what data we need? : latitude, longitude
    3. throwing to the preprocessed list to the new created list 
    '''


if __name__ == "__main__":
    print("\nRunning Novatel Pwrpak7 GPS logger....")
    
    verify_receiver_is_reachable(config["remote_host"])

    client_socket, receiver_replies_during_config = configure_receiver(config["remote_host"], config["remote_port"], receiver_commands)

    log_filename = log_data(client_socket, config["output_filename_prefix"], config["output_filename_extension"], receiver_replies_during_config)

    print("\nDone running example logger. Output from receiver {}:{} was written to file \"{}\"\n".format(config["remote_host"], config["remote_port"], log_filename))