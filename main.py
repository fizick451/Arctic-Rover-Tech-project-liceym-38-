from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import datetime  



def test():
    print('Sucssesful test')

def openPort():
    serial.setPortName(ui.comb1.currentText())
    serial.open(QIODevice.ReadWrite)
    ui.modeL.setText('ON')
    ui.modeL.setStyleSheet('color: rgb(0, 238, 255);')

def closePort():
    serial.close()
    ui.modeL.setText('OFF')
    ui.modeL.setStyleSheet('color: rgb(121, 5, 150);')
    ui.tempB.reset()
    ui.presB.reset()
    ui.tempLCD.display(0)
    ui.presLCD.display(0)

listX1 = [i for i in range(1,101)]
listY1 = [0 for i in range(100)]
listX2 = [i for i in range(1,101)]
listY2 = [0 for i in range(100)]
listX3 = [i for i in range(1,101)]
listY3 = [0 for i in range(100)]
def getData():
    pres = ui.presLCD.value()
    temp = ui.tempLCD.value()
    txt = open(r'info.txt', 'w+') 
    txt.write(currentDate+';'+str(temp)+'C'+';'+str(pres)+'mmHg\n')
    txt.close()

def ReadData():
    global listX1
    global listY1
    global listX2
    global listY2
    global listX3
    global listY3
    if not serial.canReadLine(): return
    rx = serial.readLine()
    try: 
        data = str(rx, 'utf-8').strip().split(',')
        
    
    
    
        if 'tmp' in data:
            ui.tempLCD.display(float(data[2]))
            ui.tempB.setValue(int(float(data[2])*10))
            listY1 = listY1[1:]
            listY1.append(float(data[2]))
            ui.graph.clear()
            ui.graph.plot(listX1, listY1)
        
        if 'pre' in data:
            pres = float(data[2]) * 0.75
            ui.presLCD.display(pres)
            ui.presB.setValue(int(pres)*10)
            listY2 = listY2[1:]
            listY2.append(pres)
            ui.graph1.clear()
            ui.graph1.plot(listX2, listY2)

        if 'hum' in data:
            hum = float(data[2])
            ui.humLCD.display(hum)
            ui.humB.setValue(int(hum))
            listY3 = listY3[1:]
            listY3.append(hum)
            ui.graph2.clear()
            ui.graph2.plot(listX3, listY3)

        if 'x' in data:
            x = int(data[2])/4096
            ui.xLCD.display(x)
        if 'z' in data:
            z = int(data[2])/4096
            ui.zLCD.display(z)
        if 'y' in data:
            y = int(data[2])/4096
            ui.yLCD.display(y)

        if 'rol' in data:
            rol = float(data[2])
            ui.yaLCD.display(rol)
            ui.yDial.setValue(int(rol))
        if 'pit' in data:
            rol = float(data[2])
            ui.xaLCD.display(rol)
            ui.xDial.setValue(int(rol))
        if 'az' in data:
            azimut = int(data[2])
            ui.azLCD.display(azimut)
            ui.azDial.setValue(azimut+180)

        if 'x' in data:
            x = int(data[2])/12000
            ui.xxLCD.display(x)

        if 'y' in data:
            y = int(data[2])/12000
            ui.yyLCD.display(y)

        if 'z' in data:
            y = int(data[2])/12000
            ui.zzLCD.display(z)

        if 'lng' in data:
            lon = float(data[2])
            ui.lonLCD.display(lon)

        if 'alt' in data:
            alt = float(data[2])
            ui.altLCD.display(alt)

        if 'lat' in data:
            lat = float(data[2])
            ui.latLCD.display(lat)

        if 'courseg' in data:
            cim = float(data[2])
            ui.cimLCD.display(cim)

        if 'date' in data:
            date = int(data[2])
            ui.dateLCD.display(date)

        if 'time' in data:
            time = int(data[2])
            ui.timeLCD.display(time)

        if 'sat' in data:
            sat = int(data[2])
            ui.satLCD.display(sat)
    
    except: pass
    
    

# MOVING BLOCK

currentDirection = 'NONE'
def moveToForward():
    speedVal = ui.speedSlider.value()
    
    serial.write(bytes('0,'+str(int(speedVal)),'utf-8'))
    global currentDirection
    currentDirection = '0'
    
def moveToRight():
    speedVal = ui.speedSlider.value()
    serial.write(bytes('2,'+str(int(speedVal)),'utf-8'))
    global currentDirection
    currentDirection = '2'
    
def moveToLeft():
    speedVal = ui.speedSlider.value()
    serial.write(bytes('3,'+str(int(speedVal)),'utf-8'))
    global currentDirection
    currentDirection = '3'
    
def moveToBackward():
    speedVal = ui.speedSlider.value()
    serial.write(bytes('1,'+str(int(speedVal)),'utf-8'))
    global currentDirection
    currentDirection = '1'
    
def stopMove():
    serial.write(bytes('4','utf-8'))
    global currentDirection
    currentDirection = '4'

def setSpeedToDirection():
    speedVal = ui.speedSlider.value()
    global currentDirection
    if currentDirection != 'NONE' and currentDirection != '4':
        serial.write(bytes(currentDirection+' '+str(speedVal),'utf-8'))
    ui.speedVaule.setText(str(speedVal))
    
#---------

app = QtWidgets.QApplication([])
ui = uic.loadUi('untitled.ui') 

ui.setWindowTitle('GUI Arctic Rover')
serial = QSerialPort()
serial.setBaudRate(9600)
portList = []
ports = QSerialPortInfo.availablePorts()
for port in ports:
    portList.append(port.portName())

ui.comb1.addItems(portList)
ui.writeB.clicked.connect(getData)
ui.startB.clicked.connect(openPort)
ui.stopB.clicked.connect(closePort)


# MOVING BUTTONS
#------
ui.FWB.clicked.connect(moveToForward)
ui.RB.clicked.connect(moveToRight)
ui.LB.clicked.connect(moveToLeft)
ui.BWB.clicked.connect(moveToBackward)
ui.STOPB.clicked.connect(stopMove)
ui.speedSlider.valueChanged.connect(setSpeedToDirection)

# ui.label_8.setPixmap(QPixmap('compas.jpg'))
#------

serial.readyRead.connect(ReadData)

currentDate = datetime.datetime.now().strftime('%Y-%m-%d')
ui.dateL.setText(currentDate)

ui.show()
app.exec()




















# (,pres,3452135,)