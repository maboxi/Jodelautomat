import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial#
import io
from math import sin, atan2

import struct

#SERIALCOM_USED = 'CDC'
SERIALCOM_USED = 'UART'

SERIALPORT_CDC = 'COM7'
SERIALPORT_UART = 'COM8'

CDC_FFTSTART = b'-- FFT Data START --'
CDC_FFTEND = b'-- FFT Data END --'

BAUDRATE = 2000000
UART_FLAG = b'\x00\x00\x00\x00\x00\x00\x00\x00'
UART_NUMSAMPLES = 512
UART_NUMSAMPLES_HALF = int(UART_NUMSAMPLES / 2)

FFT_NUMSAMPLES = 4096
FFT_NUMCYCLES = 33

serialHandle: serial.Serial = None
axPower = None
axPhase = None
fftMaxFrequency = FFT_NUMSAMPLES * FFT_NUMCYCLES / 2 # 35 sampling cycles per second, 4096 samples / cycle

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
    
    fftFrequencyPerBin = fftMaxFrequency / len(dataArray)
    
    plotXValues = [fftFrequencyPerBin * x for x in range(0, len(dataArray))]
    #plotYValues = [y if y > 0.0 else -y for y in dataArray]
    plotYValues = dataArray
    
    ax.clear()
    ax.set_xlim(0, fftMaxFrequency)
    ax.plot(plotXValues, plotYValues)
    

def animatePlotUART(i):
    global serialHandle
    global axPower
    global axPhase
    global ymax
    global fftMaxFrequency
    
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
    
    print(f"Received {len(receivedData)} bytes and converted them into {len(dataArray)} numbers")
    
    fftFrequencyPerBin = fftMaxFrequency / (FFT_NUMSAMPLES / 2)
    
    #(a,) = dataArray[0]
    #print(a)
    
    realFFT = [dataArray[2*i] for i in range(0, UART_NUMSAMPLES_HALF)]
    imaginaryFFT = [dataArray[2*i + 1] for i in range(0, UART_NUMSAMPLES_HALF)]
    angleFFT = [atan2(imaginaryFFT[i], realFFT[i]) for i in range(0, UART_NUMSAMPLES_HALF)]
    powerFFT = [realFFT[i] * realFFT[i] + imaginaryFFT[i] * imaginaryFFT[i] for i in range(0, UART_NUMSAMPLES_HALF)]
    
    
    plotXValues = [fftFrequencyPerBin * x for x in range(0, UART_NUMSAMPLES_HALF)]
    
    (powMin, powMax) = axPower.get_ylim()
    powMin = min(powMin, min(powerFFT))
    powMax = max(powMax, max(powerFFT))
    
    axPower.clear()
    axPower.set_xlim(0, fftFrequencyPerBin * (UART_NUMSAMPLES_HALF + 1))
    axPower.plot(plotXValues, powerFFT)
    axPower.set_ylim(powMin, powMax)
    
    (phaMin, phaMax) = axPhase.get_ylim()
    phaMin = min(phaMin, min(angleFFT))
    phaMax = max(phaMax, max(angleFFT))
    
    axPhase.clear()
    axPhase.set_xlim(0, fftFrequencyPerBin * (UART_NUMSAMPLES_HALF + 1))
    axPhase.plot(plotXValues, angleFFT)
    axPhase.set_ylim(phaMin, phaMax)
    


if __name__ == '__main__':
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
    
    fig1 = plt.figure(1)
    
    axPower = fig1.add_subplot(2, 1, 1)
    axPhase = fig1.add_subplot(2, 1, 2)
    
    axPower.plot([], [])
    axPhase.plot([], [])
    
    
    if(SERIALCOM_USED == 'UART'):
        fftFrequencyPerBin = fftMaxFrequency / FFT_NUMSAMPLES
        
        axPower.set_xlim(0, fftFrequencyPerBin * UART_NUMSAMPLES)
        axPhase.set_xlim(0, fftFrequencyPerBin * UART_NUMSAMPLES)
        
        axPower.set_ylim(-1.0, 1.0)
        axPhase.set_ylim(-1.0, 1.0)
        
        ani = animation.FuncAnimation(fig1, animatePlotUART, interval=1) # max 50 fps
    elif(SERIALCOM_USED == 'CDC'):
        ani = animation.FuncAnimation(fig1, animatePlotCDC, interval=3000)
    
    plt.show()