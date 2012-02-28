# R.A.V.E.N. base station connection interface and GUI
# Authors: Paul Martin & William Etter
# Last Modified: 03/09/11

import wx
import wx.lib.plot as plot
 
class MyForm(wx.Frame):
 
    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY, "Timer Tutorial 1",
                                   size=(500,500))
        self.panel1 = wx.Panel(self)
        # Add a panel so it looks the correct on all platforms
        data = [(0,0),(1,1),(2,2),(3,2)]
        plotter = plot.PlotCanvas(self, size=(400,300))
        line = plot.PolyLine(data, colour='red', width=1)
        marker = plot.PolyMarker(data, marker='triangle')
        gc = plot.PlotGraphics([line, marker], 'Line', 'x', 'y')
        plotter.Draw(gc, xAxis=(0,15), yAxis=(0,15))
        self.Show()
                 
 
# Run the program
if __name__ == "__main__":
    app = wx.PySimpleApp()
    frame = MyForm().Show()
    app.MainLoop()
