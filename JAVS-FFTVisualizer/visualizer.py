import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial#
import io
from math import sin

import struct

SERIALCOM_USED = 'UART'
#SERIALCOM_USED = 'CDC'

SERIALPORT_CDC = 'COM7'
SERIALPORT_UART = 'COM8'

BAUDRATE = 2000000
CDC_FFTSTART = b'-- FFT Data START --'
CDC_FFTEND = b'-- FFT Data END --'

UART_FLAG = b'\x00\x00\x00\x00\x00\x00\x00\x00'
UART_NUMSAMPLES = 512

FFT_NUMSAMPLES = 4096
FFT_NUMCYCLES = 35

def serialWaitFor(s, name):
    global serialHandle
    
    while(True):
        line = serialHandle.readline()
        if(s in line):
            print(f"Received correct {name}: {line}")
            break
        else:
            print(f"Received: '{line}'")
            pass

def animatePlotCDC(i):
    global serialHandle
    
    wait = input("waiting... press enter")
    
    serialHandle.write(b"sendfft\r\n")
    serialHandle.flush()
    print("Waiting for CDC data... ")
    
    serialWaitFor(CDC_FFTSTART, "preamble")
    
    receivedData = serialHandle.readline()
    dataStringArray = receivedData.split(b' ')
    dataStringArray = dataStringArray[:(len(dataStringArray) - 1)]
    
    serialWaitFor(CDC_FFTEND, "epilogue")
    
    print(f"Received {len(receivedData)} bytes in var of type {type(receivedData)}")
    
    
    dataArray = [float(x) for x in dataStringArray]
    
    fftFrequencyPerBin = fftSamplingFrequency / len(dataArray)
    
    plotXValues = [fftFrequencyPerBin * x for x in range(0, len(dataArray))]
    #plotYValues = [y if y > 0.0 else -y for y in dataArray]
    plotYValues = dataArray
    
    ax.clear()
    ax.set_xlim(0, fftSamplingFrequency)
    ax.plot(plotXValues, plotYValues)
    

def animatePlotUART(i):
    global serialHandle
    global ax
    global ymax
    
    print("Waiting for UART data... ")
    
    receivedData = None
    
    while(True):
        receivedData = serialHandle.read_until(UART_FLAG)
        if(len(receivedData) == (UART_NUMSAMPLES * 4 + len(UART_FLAG))):
            break
        else:
            print(f"Discarded serial read with {len(receivedData)} bytes...")
    
    # remove UART flag from data
    receivedData = receivedData[:(len(receivedData) - len(UART_FLAG))]

    # convert byte array into float array
    dataArray = struct.unpack(f"{UART_NUMSAMPLES}f", receivedData)
        
    
    print(f"Received {len(receivedData)} bytes and converted them into {len(dataArray)} floats")
    
    fftFrequencyPerBin = fftSamplingFrequency / FFT_NUMSAMPLES
    
    #(a,) = dataArray[0]
    #print(a)
    
    plotXValues = [fftFrequencyPerBin * x for x in range(0, len(dataArray))]
    plotYValues = [y if y > 0.0 else -y for y in dataArray]
    #plotYValues = dataArray
    
    ymax = max(max(dataArray), ymax, 1000.0)
    
    (xmin, xmax) = ax.get_xlim()
    ax.clear()
    ax.set_xlim(max(0, xmin), min(fftFrequencyPerBin * (len(dataArray) + 1), xmax))
    ax.set_ylim(-10.0, ymax)
    ax.plot(plotXValues, plotYValues)
    '''
    ax.clear()
    ax.plot(range(0, 100), range(0,100))
    '''
    


if __name__ == '__main__':
    global serialHandle
    global ymax
    global ax
    
    if(SERIALCOM_USED == 'UART'):
        serialHandle  = serial.Serial(port=SERIALPORT_UART, baudrate = BAUDRATE)
    elif(SERIALCOM_USED == 'CDC'):
        serialHandle  = serial.Serial(port=SERIALPORT_CDC)
    else:
        print("Unknown serial port selection (var SERIALCOM_USED)!")
        exit(1)
    
    serialHandle.set_buffer_size(65535, 128)
    #serialWrapper = io.TextIOWrapper(io.BufferedRWPair(serialHandle, serialHandle), newline='\r\n' )
    
    print("Serial connection established!")
    
    fftSamplingFrequency = FFT_NUMSAMPLES * FFT_NUMCYCLES / 2 # 35 sampling cycles per second, 4096 samples / cycle
    
    
    fig,ax = plt.subplots(1, 1)
    ax.plot([], [])

    ymax = 1.0
    
    if(SERIALCOM_USED == 'UART'):
        ani = animation.FuncAnimation(fig, animatePlotUART, interval=100)
        fftFrequencyPerBin = fftSamplingFrequency / FFT_NUMSAMPLES
        ax.set_xlim(0, fftFrequencyPerBin * UART_NUMSAMPLES)
        ax.set_ylim(-10.0, 4097.0)
    elif(SERIALCOM_USED == 'CDC'):
        ani = animation.FuncAnimation(fig, animatePlotCDC, interval=3000)
    
    plt.show()