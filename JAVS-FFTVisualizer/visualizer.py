import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial#
import io
from math import sin, atan2

import struct

#SERIALCOM_USED = 'CDC' # not supported right now!
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
axTop = None
axBot = None
fftMaxFrequency = FFT_NUMSAMPLES * FFT_NUMCYCLES / 2 # 35 sampling cycles per second, 4096 samples / cycle

FFT_NUMRECURSIVEMAX = 60
fftRecursiveMaxValues = [0] * 60
fftRecursiveMaxCounter = 0



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
    pass

def animatePlotUART(i):
    global serialHandle
    global axTop
    global axBot
    global fftMaxFrequency
    global fftRecursiveMaxValues
    global fftRecursiveMaxCounter
    
    print("\nWaiting for UART data... ")
    
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

    
    # calculate fft results from complex result data
    realFFT = [dataArray[2*i] for i in range(0, UART_NUMSAMPLES_HALF)]
    imaginaryFFT = [dataArray[2*i + 1] for i in range(0, UART_NUMSAMPLES_HALF)]
    angleFFT = [atan2(imaginaryFFT[i], realFFT[i]) for i in range(0, UART_NUMSAMPLES_HALF)]
    powerFFT = [realFFT[i] * realFFT[i] + imaginaryFFT[i] * imaginaryFFT[i] for i in range(0, UART_NUMSAMPLES_HALF)]
    
    powerMax = max(powerFFT)

    fftRecursiveMaxValues[fftRecursiveMaxCounter] = powerMax * 1.1
    fftRecursiveMaxValuesSloped = fftRecursiveMaxValues
    
    slope = 1 / (16 * FFT_NUMRECURSIVEMAX)
    
    for i in range(0, FFT_NUMRECURSIVEMAX):
        j =  (i + fftRecursiveMaxCounter) % FFT_NUMRECURSIVEMAX
        fftRecursiveMaxValuesSloped[j] = fftRecursiveMaxValuesSloped[j] * (1 - i * slope)
        
        
    fftRecursiveMaxCounter = (fftRecursiveMaxCounter + 1) % FFT_NUMRECURSIVEMAX
    
    fftRecursiveMax = max(fftRecursiveMaxValuesSloped)
    
    print(fftRecursiveMax)

    plotXValues = [fftFrequencyPerBin * x for x in range(0, UART_NUMSAMPLES_HALF)]
    

    
    axTop.clear()
    axTop.set_xlim(0, fftFrequencyPerBin * (UART_NUMSAMPLES_HALF + 1))
    axTop.plot(plotXValues, powerFFT)
    axTop.set_ylim(0, fftRecursiveMax)
    
    '''
    (phaMin, phaMax) = axPhase.get_ylim()
    phaMin = min(phaMin, min(angleFFT))
    phaMax = max(phaMax, max(angleFFT))
    
    axPhase.clear()
    axPhase.set_xlim(0, fftFrequencyPerBin * (UART_NUMSAMPLES_HALF + 1))
    axPhase.plot(plotXValues, angleFFT)
    axPhase.set_ylim(phaMin, phaMax)
    '''
    
    (_, powMax) = axBot.get_ylim()
    powMax = max(powMax, powerMax)
    
    axBot.clear()
    axBot.set_xlim(0, fftFrequencyPerBin * (UART_NUMSAMPLES_HALF + 1))
    axBot.plot(plotXValues, powerFFT)
    axBot.set_ylim(0, powMax)


if __name__ == '__main__':
    if(SERIALCOM_USED == 'UART'):
        serialHandle  = serial.Serial(port=SERIALPORT_UART, baudrate = BAUDRATE)
    elif(SERIALCOM_USED == 'CDC'):
        serialHandle  = serial.Serial(port=SERIALPORT_CDC)
    else:
        print(f"Unknown serial port selection (var SERIALCOM_USED = {SERIALCOM_USED})!")
        exit(1)
    
    serialHandle.set_buffer_size(65535, 128)
    
    print("Serial connection established!")
    
    fig1 = plt.figure(1)
    
    axTop = fig1.add_subplot(2, 1, 1)
    axBot = fig1.add_subplot(2, 1, 2)
    
    axTop.plot([], [])
    axBot.plot([], [])
    
    
    if(SERIALCOM_USED == 'UART'):
        fftFrequencyPerBin = fftMaxFrequency / FFT_NUMSAMPLES
        
        axTop.set_xlim(0, fftFrequencyPerBin * UART_NUMSAMPLES)
        axBot.set_xlim(0, fftFrequencyPerBin * UART_NUMSAMPLES)
        
        axTop.set_ylim(-1.0, 1.0)
        axBot.set_ylim(-1.0, 1.0)
        
        ani = animation.FuncAnimation(fig1, animatePlotUART, interval=1) # max 50 fps
    elif(SERIALCOM_USED == 'CDC'):
        ani = animation.FuncAnimation(fig1, animatePlotCDC, interval=3000)
    
    plt.show()