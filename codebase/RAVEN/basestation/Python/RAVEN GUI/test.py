# R.A.V.E.N. base station connection interface and GUI
# Authors: Paul Martin & William Etter
# Last Modified: 03/09/11

#!/usr/bin/python

# sizeevent.py

import wx

class SizeEvent(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title)

        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Centre()
        self.Show(True)

    def OnSize(self, event):
        self.SetTitle(str(event.GetSize()))


app = wx.App()
SizeEvent(None, 1, 'sizeevent.py')
app.MainLoop()
