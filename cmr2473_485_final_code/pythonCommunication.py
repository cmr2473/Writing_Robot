
import serial

serialPort = serial.Serial(port = "COM15", baudrate = 9600,bytesize=8,timeout=5000,stopbits=serial.STOPBITS_ONE)


serialString = ""
def clc():
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    print("")
    
def zero():
    print("exit")
def one():
    serialPort.write(('a' + '\n').encode('Ascii'))
    
    w = input("Input the width of the page in CM")
    w = w + '\n'
    serialPort.write(w.encode('Ascii'))
    
    l = input("Input the height of the page in CM")
    l = l + '\n'
    serialPort.write(l.encode('Ascii'))
    
    f = input("Input the fontWidth of the letter in CM")
    f = f + '\n'
    serialPort.write(f.encode('Ascii'))
    clc()
    print("Robot is now zeroed and should have moved to first letter location")
def two():
    serialPort.write(('b' + '\n').encode('Ascii'))
    
    string = input("input a word to write: ")
    serialPort.write((string+'\n').encode('Ascii'));
    clc()
def three():
    serialPort.write(('c'+'\n').encode('Ascii'))
def four():
    serialPort.write(('d'+'\n').encode('Ascii'))
    stop = input("send any letter to stop line following")
    serialPort.write(('l'+'\n').encode('Ascii'))
def five():
    print("invalid input")
options = {0 : zero,
           1 : one,
           2 : two,
           3 : three,
           4 : four,
           5 : five,

    }



    
        
x = 10
while x!='0':
        
        print("")
        print("")
        if(serialPort.in_waiting >0):
            serialString = serialPort.readline()
            print(serialString.decode('Ascii'))
        
        print("Choose a command:")
        print("\t Options")
        print("\t\t1) Set page width and height, and start a writing")
        print("\t\t2) Set write a string")
        print("\t\t3) Goto a new line")
        print("\t\t4) line follow")
        print("\t\t0) Quit")
        
        x = input("Enter the number of your command:")
        clc()
        if x != '0' and x!= '1' and x!= '2' and x!='3' and x!='4':
            x=5
        options[int(x)]()
        
serialPort.close()