# R.A.V.E.N. base station connection interface and GUI
# Authors: Paul Martin & William Etter
# Last Modified: 03/09/11

# -- Import Modules --
import wx
import wx.lib.plot as plot
import serial
import csv
import time


# -- Custom Frame --
class TopFrame(wx.Frame):
  
    def __init__(self, parent, title):
        super(TopFrame, self).__init__(parent, title=title, pos=(20,20))
        
        # Wireless ID constants
        self.LEAD_SH1 = 0x00
        self.LEAD_SH2 = 0x13
        self.LEAD_SH3 = 0xA2
        self.LEAD_SH4 = 0x00
        self.LEAD_SL1 = 0x40
        self.LEAD_SL2 = 0x47
        self.LEAD_SL3 = 0x51
        self.LEAD_SL4 = 0xDC
        self.FOL1_SH1 = 0x00
        self.FOL1_SH2 = 0x13
        self.FOL1_SH3 = 0xA2
        self.FOL1_SH4 = 0x00
        self.FOL1_SL1 = 0x40
        self.FOL1_SL2 = 0x47
        self.FOL1_SL3 = 0x2C
        self.FOL1_SL4 = 0x6E
        self.BASE_SH1 = 0x00
        self.BASE_SH2 = 0x13
        self.BASE_SH3 = 0xA2
        self.BASE_SH4 = 0x00
        self.BASE_SL1 = 0x40
        self.BASE_SL2 = 0x68
        self.BASE_SL3 = 0xB4
        self.BASE_SL4 = 0xBF

        # logger
        self.outputFile = open("log.csv","w")

        # on close
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # variables
        self.com_status = 0
        self.ir_coords = [(-10,0),(-10,0)]
        self.ir_cent_coords = [(-10,0),(-10,0)]
        self.counter1 = 2
        self.counter2 = 2
        self.serialPort = 0
        self.lead_rf_ps = 0
        self.foll_rf_ps = 0
        self.lead_rf_ps_old = 0
        self.foll_rf_ps_old = 0

        # buffers for plotting
        self.yawBuff = [(0,0),(1,0)]
        self.pitchBuff = [(0,0),(1,0)]
        self.rollBuff = [(0,0),(1,0)]

        self.disp_x_Buff = [(0,0),(1,0)]
        self.disp_y_Buff = [(0,0),(1,0)]
        self.disp_z_Buff = [(0,0),(1,0)]

        self.son_front_Buff = [(0,0),(1,0)]
        self.son_back_Buff = [(0,0),(1,0)]
        self.son_left_Buff = [(0,0),(1,0)]
        self.son_right_Buff = [(0,0),(1,0)]
        self.son_up_Buff = [(0,0),(1,0)]
        self.son_down_Buff = [(0,0),(1,0)]

        # timer for mainloop
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.timer.Start(20) # 50/sec

        # timer for rf throughput metrics
        self.rf_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRFTimer, self.rf_timer)
        self.rf_timer.Start(1000)

        # 3 horizontal panels from top to bottom
        self.displacement_frame = wx.Frame(None, title="Calculated Displacement", size=(400,400), pos=(430,150))
        self.ir_frame = wx.Frame(None, title="Infrared Camera", size=(400,400), pos=(20,150))
        self.sonar_frame = wx.Frame(None, title="Sonar Distance", size=(400,400), pos=(840,150))
        
        self.pnl_1 = wx.Panel(self, id=-1)

        # move these frames to panels
        pnl_vbox = wx.BoxSizer(wx.VERTICAL)   
        
        ###### PANEL 1 #########
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

        ## -- RF Throughput --
        rf_box = wx.StaticBox(self.pnl_1, label="Link Rates")
        rf_vbox = wx.StaticBoxSizer(rf_box, wx.VERTICAL)
        # leader
        self.lead_rf = wx.StaticText(self.pnl_1, label="0.0")
        self.lead_rf.SetForegroundColour('blue')
        rf_hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        rf_hbox1.Add(wx.StaticText(self.pnl_1, label="Leader   :"), flag=wx.LEFT|wx.RIGHT, border=5)
        rf_hbox1.Add(self.lead_rf, flag=wx.LEFT|wx.RIGHT, border=5)
        # follower
        self.foll_rf = wx.StaticText(self.pnl_1, label="0.0")
        self.foll_rf.SetForegroundColour('blue')
        rf_hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        rf_hbox2.Add(wx.StaticText(self.pnl_1, label="Follower :"), flag=wx.LEFT|wx.RIGHT, border=5)
        rf_hbox2.Add(self.foll_rf, flag=wx.LEFT|wx.RIGHT, border=5)
        # add to vbox and overall sizer
        rf_vbox.Add(rf_hbox1, flag=wx.TOP|wx.BOTTOM, border=5)
        rf_vbox.Add(rf_hbox2, flag=wx.TOP|wx.BOTTOM, border=5)
        pnl_1_sizer.Add(rf_vbox, flag=wx.EXPAND|wx.ALL, border=5)


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
        
        ## -- Wireless Commands--
        self.spin_but = wx.Button(self.pnl_1, label="Spinup", size=(100,-1))
        self.follow_but = wx.Button(self.pnl_1, label="Follow", size=(100,-1))
        self.land_but = wx.Button(self.pnl_1, label="Autoland", size=(100,-1))
        self.e_stop = wx.Button(self.pnl_1, label="E-STOP", size=(100,-1))
        pnl_1_sizer.Add(self.spin_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.follow_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.land_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.e_stop, flag=wx.ALL, border=5)
        self.spin_but.Bind(wx.EVT_BUTTON, self.OnSpin)
        self.follow_but.Bind(wx.EVT_BUTTON, self.OnFollow)
        self.land_but.Bind(wx.EVT_BUTTON, self.OnLand)
        self.e_stop.Bind(wx.EVT_BUTTON, self.OnStop)

        ## -- PID Tuning Buttons --
        self.pid_up_but = wx.Button(self.pnl_1, label="PID up", size=(100,-1))
        self.pid_down_but = wx.Button(self.pnl_1, label="PID down", size=(100,-1))
        pnl_1_sizer.Add(self.pid_up_but, flag=wx.ALL, border=5)
        pnl_1_sizer.Add(self.pid_down_but, flag=wx.ALL, border=5)
        self.pid_up_but.Bind(wx.EVT_BUTTON, self.OnPIDup)
        self.pid_down_but.Bind(wx.EVT_BUTTON, self.OnPIDdown)

        pnl_vbox.Add(self.pnl_1)

       
        
        ####### PLOT FRAMES  ##########
        # ir camera
        self.ir_canvas = plot.PlotCanvas(self.ir_frame)
        markers = plot.PolyMarker(self.ir_coords, legend='', colour='blue',marker='circle', size=2)
        gc = plot.PlotGraphics([markers], 'IR Camera', 'X Axis', 'Y Axis')
        self.ir_canvas.Draw(gc, xAxis=(0,1024), yAxis=(0,1024))
        self.ir_frame.Show()

        # displacements
        self.disp_canvas = plot.PlotCanvas(self.displacement_frame)
        x_line = plot.PolyLine(self.disp_x_Buff, colour='red', width=1)
        y_line = plot.PolyLine(self.disp_x_Buff, colour='blue', width=1)
        z_line = plot.PolyLine(self.disp_x_Buff, colour='green', width=1)
        gc = plot.PlotGraphics([x_line, y_line, z_line], 'Displacement', 'Time','Distance (cm)')
        self.disp_canvas.Draw(gc, xAxis=(0,100), yAxis=(-300,300))
        self.displacement_frame.Show()
        
        # sonar
        self.sonar_canvas = plot.PlotCanvas(self.sonar_frame)
        x_line = plot.PolyLine(self.son_front_Buff, colour='red', width=1)
        y_line = plot.PolyLine(self.son_back_Buff, colour='blue', width=1)
        z_line = plot.PolyLine(self.son_left_Buff, colour='green', width=1)        
        a_line = plot.PolyLine(self.son_right_Buff, colour='red', width=1)
        b_line = plot.PolyLine(self.son_up_Buff, colour='blue', width=1)
        c_line = plot.PolyLine(self.son_down_Buff, colour='green', width=1)
        gc = plot.PlotGraphics([x_line, y_line, z_line, a_line, b_line, c_line], 'Sonar Ranges', 'Time','Distance (cm)')
        self.sonar_canvas.Draw(gc, xAxis=(0,100), yAxis=(-100,100))
        self.sonar_frame.Show()

        ######### CLEAN UP ############
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

    """ sending commands """
    def OnSpin(self, event):
        self.SendRF('follower', 0xA1,1)
    def OnLand(self, event):
        self.SendRF('follower', 0xB2,1)
    def OnStop(self, event):
        self.SendRF('follower', 0xC3,2)
    def OnFollow(self, event):
        self.SendRF('leader', 0xD4,1)
        time.sleep(0.5)
        self.SendRF('follower', 0xD4,1)
    def OnPIDup(self, event):
        self.SendRF('follower', 0xE5,1)
    def OnPIDdown(self, event):
        self.SendRF('follower', 0xF6,1)

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

                    # -- Payload --
                    unitID = ord(self.serialPort.read(1))
                    if unitID == 1:
                        """ THIS IS FOR A LEADER """
                        self.lead_rf_ps = self.lead_rf_ps + 1

                         # attitude
                        yaw = 360.0*ord(self.serialPort.read(1))/255.0
                        if yaw > 180:
                            yaw = -360+yaw
                        pitch = 360.0*ord(self.serialPort.read(1))/255.0
                        if pitch > 180:
                            pitch = -360+pitch
                        roll = 360.0*ord(self.serialPort.read(1))/255.0
                        if roll > 180:
                            roll = -360+roll
                        # battery
                        bat = ord(self.serialPort.read(1))/10.0

                        # checksum
                        self.serialPort.read(1)

                        # update battery label
                        self.lead_bat.SetLabel(str(bat))

                        # log values to '1' -- follower
                        #self.outputFile.write('1')
                        #self.outputFile.write(',')
                        #self.outputFile.write(str(yaw))
                        #self.outputFile.write(',')
                        #self.outputFile.write(str(pitch))
                        #self.outputFile.write(',')
                        #self.outputFile.write(str(roll))
                        #self.outputFile.write(',')
                        #self.outputFile.write('\n')

                        #print "-1-"
                        #print str(yaw)
                        #print str(pitch)
                        #print str(roll)
                        
                    elif unitID == 2:
                        """ THIS IS FOR A FOLLOWER """
                        self.foll_rf_ps = self.foll_rf_ps + 1

                        # attitude
                        yaw = 360.0*ord(self.serialPort.read(1))/255.0
                        if yaw > 180:
                            yaw = -360+yaw
                        pitch = 360.0*ord(self.serialPort.read(1))/255.0
                        if pitch > 180:
                            pitch = -360+pitch
                        roll = 360.0*ord(self.serialPort.read(1))/255.0
                        if roll > 180:
                            roll = -360+roll

                        #print "-2-"
                        #print str(yaw)
                        #print str(pitch)
                        #print str(roll)

                        # battery
                        bat = ord(self.serialPort.read(1))/10.0

                        # ir camera
                        ir_1_x = 1024*ord(self.serialPort.read(1))/255;
                        ir_1_y = 1024*ord(self.serialPort.read(1))/255;
                        ir_2_x = 1024*ord(self.serialPort.read(1))/255;
                        ir_2_y = 1024*ord(self.serialPort.read(1))/255;
                        ir_cent_x = 1024*ord(self.serialPort.read(1))/255;
                        ir_cent_y = 1024*ord(self.serialPort.read(1))/255;

                        # displacement - two bytes each
                        disp_xh = ord(self.serialPort.read(1))
                        disp_xl = ord(self.serialPort.read(1))
                        disp_x = ((disp_xh << 8) | disp_xl) - 1000
                        disp_yh = ord(self.serialPort.read(1))
                        disp_yl = ord(self.serialPort.read(1))
                        disp_y = ((disp_yh << 8) | disp_yl) - 1000
                        disp_zh = ord(self.serialPort.read(1))
                        disp_zl = ord(self.serialPort.read(1))
                        disp_z = ((disp_zh << 8) | disp_zl) - 1000

                        #print "displace:"
                        #print disp_x
                        #print disp_y
                        #print disp_z

                        # sonar
                        son_front = ord(self.serialPort.read(1))
                        son_back = ord(self.serialPort.read(1))
                        son_left = ord(self.serialPort.read(1))
                        son_right = ord(self.serialPort.read(1))
                        son_up = ord(self.serialPort.read(1))
                        son_down = ord(self.serialPort.read(1))

                        # pid term
                        pid = ord(self.serialPort.read(1))
                        print str(pid)

                        # checksum
                        thisByte = self.serialPort.read(1)
                        
                        # update battery label
                        self.foll_bat.SetLabel("N/A")

                        # Moving Displacement Plot
                        if self.counter2 >= 100:
                            temp = self.disp_x_Buff.pop(0)
                        self.disp_x_Buff.append((self.counter2,disp_x))
                        if self.counter2 >= 100:
                            temp = self.disp_y_Buff.pop(0)
                        self.disp_y_Buff.append((self.counter2,disp_y))
                        if self.counter2 >= 100:
                            temp = self.disp_z_Buff.pop(0)
                        self.disp_z_Buff.append((self.counter2,disp_z))

                        # Moving Sonar Plot
                        if self.counter2 >= 100:
                            temp = self.son_front_Buff.pop(0)
                        self.son_front_Buff.append((self.counter2,son_front))
                        if self.counter2 >= 100:
                            temp = self.son_back_Buff.pop(0)
                        self.son_back_Buff.append((self.counter2,son_back))
                        if self.counter2 >= 100:
                            temp = self.son_left_Buff.pop(0)
                        self.son_left_Buff.append((self.counter2,son_left))
                        if self.counter2 >= 100:
                            temp = self.son_right_Buff.pop(0)
                        self.son_right_Buff.append((self.counter2,son_right))
                        if self.counter2 >= 100:
                            temp = self.son_up_Buff.pop(0)
                        self.son_up_Buff.append((self.counter2,son_up))
                        if self.counter2 >= 100:
                            temp = self.son_down_Buff.pop(0)
                        self.son_down_Buff.append((self.counter2,son_down))

                        # update plots if com port is open
                        if self.com_status == 1:
                            # ir camera
                            self.ir_coords = ((ir_1_x,ir_1_y),(ir_2_x,ir_2_y))
                            self.ir_cent_coords = ((ir_cent_x,ir_cent_y),(ir_cent_x,ir_cent_y))
                            markers = plot.PolyMarker(self.ir_coords, legend='', colour='blue',marker='circle', size=1.5)
                            center_mark = plot.PolyMarker(self.ir_cent_coords, legend='', colour='red', marker='cross', size=3)
                            gc = plot.PlotGraphics([markers, center_mark])
                            self.ir_canvas.Draw(gc, xAxis=(0,1024), yAxis=(0,1024))

                            # displacement
                            x_line = plot.PolyLine(self.disp_x_Buff, colour='red', width=2)
                            y_line = plot.PolyLine(self.disp_y_Buff, colour='blue', width=2)
                            z_line = plot.PolyLine(self.disp_z_Buff, colour='green', width=2)
                            gc = plot.PlotGraphics([x_line, y_line, z_line])
                            self.disp_canvas.Draw(gc, xAxis=(self.counter2-100,self.counter2+20), yAxis=(-300,300))

                            # sonar
                            x_line = plot.PolyLine(self.son_front_Buff, colour='black', width=2)
                            y_line = plot.PolyLine(self.son_back_Buff, colour='brown', width=2)
                            z_line = plot.PolyLine(self.son_left_Buff, colour='grey', width=2)
                            a_line = plot.PolyLine(self.son_right_Buff, colour='red', width=2)
                            b_line = plot.PolyLine(self.son_up_Buff, colour='orange', width=2)
                            c_line = plot.PolyLine(self.son_down_Buff, colour='green', width=2)
                            gc = plot.PlotGraphics([x_line, y_line, z_line, a_line, b_line, c_line])
                            self.sonar_canvas.Draw(gc, xAxis=(self.counter2-100,self.counter2+20), yAxis=(0,256))
                        
                            # log values to '2' -- follower
                            self.outputFile.write('2')
                            self.outputFile.write(',')
                            self.outputFile.write(str(yaw))
                            self.outputFile.write(',')
                            self.outputFile.write(str(pitch))
                            self.outputFile.write(',')
                            self.outputFile.write(str(roll))
                            self.outputFile.write(',')
                            self.outputFile.write(str(ir_1_x))
                            self.outputFile.write(',')
                            self.outputFile.write(str(ir_1_y))
                            self.outputFile.write(',')
                            self.outputFile.write(str(ir_2_x))
                            self.outputFile.write(',')
                            self.outputFile.write(str(ir_2_y))
                            self.outputFile.write(',')
                            self.outputFile.write(str(disp_x))
                            self.outputFile.write(',')
                            self.outputFile.write(str(disp_y))
                            self.outputFile.write(',')
                            self.outputFile.write(str(disp_z))
                            self.outputFile.write('\n')
                        
                        # increment counter 2
                        self.counter2 = self.counter2 + 1

                    """ END OF SWITCH CASE """

                    # parsed all of packet, flush the rest so we have the most recent
                    self.serialPort.flushInput()
                    



    def SendRF(self, recipient, cmd, num):
        checksum = 0
        frameID = 1
        packet = []
        packet.append(0x7E)
        packet.append(0x00)
        packet.append(0x10) # length = 16
        packet.append(0x10) # TX request
        checksum = checksum + 0x10
        packet.append(frameID)
        checksum = checksum + frameID
        
        if recipient == 'leader':
            packet.append(self.LEAD_SH1)
            packet.append(self.LEAD_SH2)
            packet.append(self.LEAD_SH3)
            packet.append(self.LEAD_SH4)
            packet.append(self.LEAD_SL1)
            packet.append(self.LEAD_SL2)
            packet.append(self.LEAD_SL3)
            packet.append(self.LEAD_SL4)
            checksum = checksum + self.LEAD_SH1 + self.LEAD_SH2 + self.LEAD_SH3 + self.LEAD_SH4
            checksum = checksum + self.LEAD_SL1 + self.LEAD_SL2 + self.LEAD_SL3 + self.LEAD_SL4
        elif recipient == 'follower':
            packet.append(self.FOL1_SH1)
            packet.append(self.FOL1_SH2)
            packet.append(self.FOL1_SH3)
            packet.append(self.FOL1_SH4)
            packet.append(self.FOL1_SL1)
            packet.append(self.FOL1_SL2)
            packet.append(self.FOL1_SL3)
            packet.append(self.FOL1_SL4)
            checksum = checksum + self.FOL1_SH1 + self.FOL1_SH2 + self.FOL1_SH3 + self.FOL1_SH4
            checksum = checksum + self.FOL1_SL1 + self.FOL1_SL2 + self.FOL1_SL3 + self.FOL1_SL4
        else:
            raise NameError('Incorrect Destinatioan Requested')
        
        packet.append(0xFF)
        checksum = checksum + 0xFF
        packet.append(0xFE)
        checksum = checksum + 0xFE
        packet.append(0x00)
        packet.append(0x00)
        # unit id:
        packet.append(0x00)
        checksum = checksum + 0x00
        # payload:
        packet.append(cmd)
        checksum = checksum + cmd
        # checksum
        packet.append(0xFF - (checksum & 0xFF))

        # send to XBee
        if self.com_status == 1:
            for i in range(num):
                self.serialPort.write(bytearray(packet))

    ########## MAIN LOOP ######
    def OnTimer(self, event):
        self.ParseRF()
    def OnRFTimer(self, event):
        self.lead_rf.SetLabel(str((self.lead_rf_ps + self.lead_rf_ps_old)/2.0))
        self.foll_rf.SetLabel(str((self.foll_rf_ps + self.foll_rf_ps_old)/2.0))
        self.lead_rf_ps_old = self.lead_rf_ps
        self.foll_rf_ps_old = self.foll_rf_ps
        self.lead_rf_ps = 0
        self.foll_rf_ps = 0
    def OnClose(self, event):
        self.timer.Stop()
        self.Destroy()


# -- For Standalone Operation --
if __name__ == '__main__':
  
    app = wx.App(False)
    TopFrame(None, title='R.A.V.E.N. Base Station')
    app.MainLoop()
