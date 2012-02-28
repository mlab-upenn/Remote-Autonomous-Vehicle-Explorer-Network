# R.A.V.E.N. base station connection interface and GUI
# Authors: Paul Martin & William Etter
# Last Modified: 03/09/11

# -- Import Modules --
import wx
import wx.lib.plot as plot
import serial
import csv


# -- Custom Frame --
class TopFrame(wx.Frame):
  
    def __init__(self, parent, title):
        super(TopFrame, self).__init__(parent, title=title, pos=(20,20))

        # logger
        self.outputFile = open("log.csv","w")

        # on close
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # variables
        self.com_status = 0
        self.ir_coords = [(100,100),(200,100)]
        self.counter = 2
        self.gp1Buff = [(0,0),(1,1)]
        self.gp2Buff = [(0,0),(1,1)]
        self.gp3Buff = [(0,0),(1,1)]
        self.serialPort = 0

        # timer for mainloop
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.timer.Start(20) # 50/sec

        # 3 horizontal panels from top to bottom
        self.pnl_1 = wx.Panel(self, id=-1)
        self.ir_frame = wx.Frame(None, title="Perceived IR", size=(400,400), pos=(850,150))
        self.gyro_frame = wx.Frame(None, title="Rate Gyros", size=(1400,700), pos=(20,150))
        pnl_vbox = wx.BoxSizer(wx.VERTICAL)   
        
        ###### MAIN FRAME #########
        pnl_1_sizer = wx.BoxSizer(wx.HORIZONTAL)

        ## -- Serial Buttons --
        ser_box = wx.StaticBox(self.pnl_1, label="Serial Port");
        ser_vbox = wx.StaticBoxSizer(ser_box,wx.VERTICAL)
        self.open_but = wx.Button(self.pnl_1, label="Open")
        self.close_but = wx.Button(self.pnl_1, label="Close")
        ser_vbox.Add(self.open_but, flag=wx.LEFT|wx.RIGHT|wx.BOTTOM, border=5)
        ser_vbox.Add(self.close_but, flag=wx.LEFT|wx.RIGHT, border=5)
        self.open_but.Bind(wx.EVT_BUTTON, self.OnOpenSerial)
        self.close_but.Bind(wx.EVT_BUTTON, self.OnCloseSerial)
        
        pnl_1_sizer.Add(ser_vbox, flag=wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT|wx.BOTTOM, border=5) 
        
        ## -- Flight Mode --
        mode_box = wx.StaticBox(self.pnl_1, label="Flight Mode")
        mode_vbox = wx.StaticBoxSizer(mode_box, wx.VERTICAL)
        self.mode_txt = wx.StaticText(self.pnl_1, label="<  Offline  >")
        self.mode_txt.SetForegroundColour(wx.Color(255,140,0))
        mode_vbox.Add(self.mode_txt, flag=wx.LEFT|wx.RIGHT, border=5)
        pnl_1_sizer.Add(mode_vbox, flag=wx.EXPAND|wx.ALL, border=5)
        
        ## -- Battery Information --
        batt_box = wx.StaticBox(self.pnl_1, label="Battery Levels")
        batt_vbox = wx.StaticBoxSizer(batt_box, wx.VERTICAL)
        # leader
        self.lead_bat = wx.StaticText(self.pnl_1, label="0.0")
        self.lead_bat.SetForegroundColour('red')
        batt_hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        batt_hbox1.Add(wx.StaticText(self.pnl_1, label="Leader   :"), flag=wx.LEFT|wx.RIGHT, border=5)
        batt_hbox1.Add(self.lead_bat, flag=wx.LEFT|wx.RIGHT, border=5)
        # follower
        self.foll_bat = wx.StaticText(self.pnl_1, label="0.0")
        self.foll_bat.SetForegroundColour('red')
        batt_hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        batt_hbox2.Add(wx.StaticText(self.pnl_1, label="Follower :"), flag=wx.LEFT|wx.RIGHT, border=5)
        batt_hbox2.Add(self.foll_bat, flag=wx.LEFT|wx.RIGHT, border=5)
        # add to vbox and overall sizer
        batt_vbox.Add(batt_hbox1, flag=wx.TOP|wx.BOTTOM, border=5)
        batt_vbox.Add(batt_hbox2, flag=wx.TOP|wx.BOTTOM, border=5)
        pnl_1_sizer.Add(batt_vbox, flag=wx.EXPAND|wx.ALL, border=5)

        self.pnl_1.SetSizer(pnl_1_sizer)

        ## -- Plot Launchers --
        self.GP1_status = 0
        self.GP1_but = wx.Button(self.pnl_1, label="General 1:3", size=(100,-1))
        self.GP1_but.SetBackgroundColour(wx.BLACK)
        self.GP1_but.Bind(wx.EVT_BUTTON, self.OnGP1)
        self.GP2_status = 0
        self.GP2_but = wx.Button(self.pnl_1, label="General 4:6", size=(100,-1))
        self.GP2_but.SetBackgroundColour(wx.BLACK)
        self.GP2_but.Bind(wx.EVT_BUTTON, self.OnGP2)
        pnl_1_sizer.Add(self.GP1_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.GP2_but, flag=wx.ALL, border=5)
        
        ## -- Wireless Commands--
        self.spin_but = wx.Button(self.pnl_1, label="Spinup", size=(100,-1))
        self.land_but = wx.Button(self.pnl_1, label="Autoland", size=(100,-1))
        self.e_stop = wx.Button(self.pnl_1, label="E-STOP", size=(100,-1))
        pnl_1_sizer.Add(self.spin_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.land_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.e_stop, flag=wx.ALL, border=5)

        pnl_vbox.Add(self.pnl_1)

       
        
        ####### IR Camera  ##########
        ir_canvas = plot.PlotCanvas(self.ir_frame, size=(400,400))
        markers = plot.PolyMarker(self.ir_coords, legend='', colour='blue',marker='circle', size=2)
        gc = plot.PlotGraphics([markers], 'IR Camera', 'X Axis', 'Y Axis')
        ir_canvas.Draw(gc, xAxis=(0,1024), yAxis=(0,768))
        #self.ir_frame.Show()

        ######## Rate Gyros ###########
        self.gyro_canvas = plot.PlotCanvas(self.gyro_frame, size=(800,400))
        x_line = plot.PolyLine(self.gp1Buff, colour='red', width=1)
        y_line = plot.PolyLine(self.gp2Buff, colour='blue', width=1)
        z_line = plot.PolyLine(self.gp3Buff, colour='green', width=1)
        x_mark = plot.PolyMarker(self.gp1Buff, marker='circle')
        y_mark = plot.PolyMarker(self.gp2Buff, marker='circle')
        z_mark = plot.PolyMarker(self.gp3Buff, marker='circle')
        gc = plot.PlotGraphics([x_line, x_mark, y_line, y_mark, z_line, z_mark], 'Rate Gyros', 'Time','rate (dps)')
        self.gyro_canvas.Draw(gc, xAxis=(0,100), yAxis=(-100,100))

        # clean up
        self.SetSizerAndFit(pnl_vbox)
        self.Show()

    ######### EVENTS ##########
    def OnOpenSerial(self, event):
        self.mode_txt.SetLabel("< Opening... >")
        self.mode_txt.SetForegroundColour(wx.RED)
        try:
            self.serialPort = serial.Serial("/dev/tty.usbserial-A600eLf3", baudrate=115200)
            self.mode_txt.SetLabel("< Open >")
            self.mode_txt.SetForegroundColour(wx.GREEN)
            self.com_status = 1
        except:
            self.mode_txt.SetLabel("< No Port >")
            self.mode_txt.SetForegroundColour(wx.RED)
    
    def OnCloseSerial(self, event):
        self.mode_txt.SetLabel("< Closing... >")
        self.mode_txt.SetForegroundColour(wx.RED)
        try:
            self.serialPort.close()
            self.mode_txt.SetLabel("< Offline >")
            self.mode_txt.SetForegroundColour(wx.Color(255,140,0))
        except:
            self.mode_txt.SetLabel("< Offline >")
            self.mode_txt.SetForegroundColour(wx.Color(255,140,0))
        self.com_status = 0

    def OnGP1(self, event):
        if self.GP1_status == 0:
            self.GP1_status = 1
            self.GP1_but.SetBackgroundColour(wx.GREEN)
            self.gyro_frame.Show(True)
        else:
            self.GP1_status = 0
            self.GP1_but.SetBackgroundColour(wx.BLACK)
            self.gyro_frame.Show(False)
    def OnGP2(self, event):
        if self.GP2_status == 0:
            self.GP2_status = 1
            self.GP2_but.SetBackgroundColour(wx.GREEN)
        else:
            self.GP2_status = 0
            self.GP2_but.SetBackgroundColour(wx.BLACK)
    #def OnSpinup(self, event):
        

    ####### SUBROUTINES #######
    def ParseRF(self):
        if self.com_status == 1:
            # temp byte
            thisByte = 0
            packetFound = 0
            # if nothing here
            if self.serialPort.inWaiting() == 0:
                return
            else:
                while self.serialPort.inWaiting() > 0:
                    # bytes in buffer, find full packet
                    thisByte = self.serialPort.read(1)
                    if thisByte == '\x7E':
                        self.serialPort.read(2) #length high and low
                        packetT = self.serialPort.read(1)
                        if packetT == '\x90':
                            packetFound = 1
                            break
                if packetFound == 1:
                    # we found a packet, parse
                    thisByte = self.serialPort.read(11) # don't care
                    unitID = self.serialPort.read(1)
                    yaw = 360.0*ord(self.serialPort.read(1))/255.0
                    if yaw > 180:
                        yaw = -360+yaw
                    pitch = 360.0*ord(self.serialPort.read(1))/255.0
                    if pitch > 180:
                        pitch = -360+pitch
                    roll = 360.0*ord(self.serialPort.read(1))/255.0
                    if roll > 180:
                        roll = -360+roll
                    bat = ord(self.serialPort.read(1))/10.0
                    gx = ord(self.serialPort.read(1))-122
                    gy = ord(self.serialPort.read(1))-122
                    gz = ord(self.serialPort.read(1))-122
                    yaw_D = ord(self.serialPort.read(1))-122
                    pitch_D = ord(self.serialPort.read(1))-122
                    roll_D = ord(self.serialPort.read(1))-122
                    alt = ord(self.serialPort.read(1))/100.0

                    # Moving Plot 1
                    if self.counter >= 100:
                        temp = self.gp1Buff.pop(0)
                    self.gp1Buff.append((self.counter,yaw))
                    if self.counter >= 100:
                        temp = self.gp2Buff.pop(0)
                    self.gp2Buff.append((self.counter,pitch))
                    if self.counter >= 100:
                        temp = self.gp3Buff.pop(0)
                    self.gp3Buff.append((self.counter,roll))

                    # write altitude as d_gain
                    self.foll_bat.SetLabel(str(alt))

                    # log values
                    self.outputFile.write(str(yaw))
                    self.outputFile.write(',')
                    self.outputFile.write(str(pitch))
                    self.outputFile.write(',')
                    self.outputFile.write(str(roll))
                    self.outputFile.write('\n')

                    # parsed all of packet, flush the rest so we have the most recent
                    self.serialPort.flushInput()
                    
                    self.lead_bat.SetLabel(str(bat))
                    
                    if self.GP1_status == 1:
                        x_line = plot.PolyLine(self.gp1Buff, colour='red', width=2)
                        y_line = plot.PolyLine(self.gp2Buff, colour='blue', width=2)
                        z_line = plot.PolyLine(self.gp3Buff, colour='green', width=2)
                        x_mark = plot.PolyMarker(self.gp1Buff, marker='circle', fillstyle=wx.TRANSPARENT)
                        y_mark = plot.PolyMarker(self.gp2Buff, marker='circle', fillstyle=wx.TRANSPARENT)
                        z_mark = plot.PolyMarker(self.gp3Buff, marker='circle', fillstyle=wx.TRANSPARENT)
                        gc = plot.PlotGraphics([x_line, y_line, z_line], 'Rate Gyros', 'Time','rate (dps)')
                        #gc = plot.PlotGraphics([x_line, x_mark], 'Rate Gyros', 'Time', 'rate (dps)')
                        self.gyro_canvas.Draw(gc, xAxis=(self.counter-100,self.counter+20), yAxis=(-100,100))
                    self.counter = self.counter+1

    ########## MAIN LOOP ######
    def OnTimer(self, event):
        self.ParseRF()
    def OnClose(self, event):
        self.timer.Stop()
        self.Destroy()


# -- For Standalone Operation --
if __name__ == '__main__':
  
    app = wx.App(False)
    TopFrame(None, title='R.A.V.E.N. Base Station')
    app.MainLoop()
