import serial
import pandas as pd
import tkinter as tk
import os
import struct

start_byte = bytes([0xFE])
package_len = 23
serial_speed = 115200
serial_port = '/dev/cu.usbserial-110'

columns = ['Pinky', 'Ring', 'Middle', 'Pointer', 'Thumb', 'Q1', 'Q2', 'Q3', 'SW0', 'SW1', 'Letter']

file_path = "data.csv"

package_command = bytes([0xFE, 0x50])

package = []

ser = serial.Serial(port= serial_port, baudrate= serial_speed, parity= serial.PARITY_NONE, stopbits= serial.STOPBITS_ONE, timeout= 1)

df = pd.DataFrame(columns=columns)

sampleCount = 0

def saveParams():
    if (len(textField.get()) > 0):

        global processBtn

        processBtn.configure(state='disabled')

        letter = textField.get()[0].lower()

        if (letter.isdigit()) :
            textField.configure(bg='red')
            return

        textField.configure(bg='green')

        global ser

        ser.write(package_command)
        corrupted = True

        # Try to keep reading to see if the first byte read is 
        while (corrupted == True):
            s = ser.read(24); # Read the next 24 bytes

            if s[0] == start_byte[0]:
                corrupted = False
        
        global package

        for i in range(1, 10, 2):
            package.append(int.from_bytes(s[i:i+2], 'little', signed="False"))

        for i in range(11, 23, 4):
            package.append(struct.unpack('f', s[i:i+4])[0])

        package.append(s[23] & 0x1)
        package.append((s[23] & 0x2) >> 1)

        package.append(letter)  

        print(package)

        df.loc[len(df)] = package

        global sampleCount, counter

        sampleCount = sampleCount + 1
        counter.config(text="Number of Samples: " + str(sampleCount))

        package.clear()

        processBtn.configure(state='normal')

    else:
        textField.configure(bg='red')

def save():

    global df, sampleCount

    if os.path.isfile(file_path):
        print('Saved to Current File')
        df.to_csv(file_path, mode='a', index=False, header=False)
    else:
        print('New File Created')
        df.to_csv(file_path, index=False)

    df = df[0:0] 
    sampleCount = 0
    counter.config(text="Number of Samples: " + str(sampleCount))

def delete():

    global df

    if (len(df) >= 1):
        df.drop(labels=[(len(df)-1)], inplace=True)

        global sampleCount

        sampleCount = sampleCount - 1
        counter.config(text="Number of Samples: " + str(sampleCount))

win = tk.Tk()
win.title('Gathering ML Data')
win.geometry('500x150')

counter = tk.Label(win, text="Number of Samples: " + str(sampleCount))
counter.grid(column=0, row=1, columnspan=2)

lbl = tk.Label(win, text="Saving data for ML learning")
lbl.grid(column=0, row=0, columnspan=2)

textField = tk.Entry(win, width=50, justify='center')
textField.grid(column=0, row=2, columnspan=2)

processBtn = tk.Button(win, text='Gather Data', command=saveParams)
processBtn.grid(column=0, row=3)

saveBtn = tk.Button(win, text='Save', command=save)
saveBtn.grid(column=1, row=3)

deleteBtn = tk.Button(win, text='Delete Last Data', command=delete)
deleteBtn.grid(column=0, row=4, columnspan=2)

win.mainloop()