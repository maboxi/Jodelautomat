import matplotlib
import serial#
import io

SERIALPORT = 'COM7'
BAUDRATE = 9600
UART_FFTSTART = '-- FFT Data START --'
UART_FFTEND = '\r\n-- FFT Data END --\r\n'

if __name__ == '__main__':
    serialHandle  = serial.Serial(port=SERIALPORT)
    serialHandle.set_buffer_size(65535, 128)
    serialWrapper = io.TextIOWrapper(io.BufferedRWPair(serialHandle, serialHandle), newline='\r\n' )
    
    print("Serial connection established!")
    

    serialWrapper.write("sendfft\r\n")
    serialWrapper.flush()
    print("Waiting for data... ")
    
    while(True):
        preamble = serialWrapper.readline()
        if(UART_FFTSTART in preamble):
            print(f"Received correct preamble: {preamble}")
            break
        else:
            print(f"Received: '{preamble}'")
    
    
    receivedData = ""
    
    while(True):
        received = serialWrapper.read()
        if(received == '\n'):
            break
        else:
            receivedData += received
    
    #receivedData = serialWrapper.readline()
    print(f"Received {len(receivedData)} bytes in var of type {type(receivedData)}")
    print(receivedData)
    
    
    
    