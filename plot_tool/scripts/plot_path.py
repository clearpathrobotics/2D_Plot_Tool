#!/usr/bin/env python

##	@package plot_tool
#	This tool is offers a ROS service that draws 2D parametric plots
#	The tool currently supports geometry_msgs::Pose and nav_msgs::Path
#	The plotting is done using pyqtgraph v0.9.8

import rospy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from nav_msgs.msg import Path 
from geometry_msgs.msg import Pose
from plot_tool.srv import *

#	This is a global graph object, we need it global so your service handlers can access the graph
graph_obj = None

##	Class CustomGraphicsWindow
#	We use this class to override the closeEvent function of the base pg.GraphicsWindow
#	This is used to reopen the window, and redraw all the content, whenever the graph window is closed.
class CustomGraphicsWindow(pg.GraphicsWindow):
	##	Function closeEvent
	#	This overrides the closeEvent function in the base class. This function is invoked automatically by QTGui when the window closes.
	#	@param ev This is the event object (i.e. window close). 
	def closeEvent(self,ev):
		# recreate the graph window
		graph_obj.win = CustomGraphicsWindow()
		graph_obj.win.setWindowTitle('Plot Tool')
		graph_obj.graph = graph_obj.win.addPlot()
		graph_obj.graph.showGrid(x=True, y=True)
		# iterate through the current plots, and readd them to the graph GUI
		for s in graph_obj.plot_tracker:
			for p in s:
				graph_obj.graph.addItem(p)

##	Class Graph_Drawer 
#	This class manages all the pyqtgraph plotting and rendering.
#	If we wish to use another backend for drawing the graphs, this class should be swapped out.
class Graph_Drawer:
	# We currently support 7 different data series each with its own color and plot storage
	# We can easily add more if needed, however we may have to define the colors as (R,G,B)
	colors=['g','r','b','c','m','y','w']
	plot_tracker=[[] for i in range(7)]
	# This is the list that tracks pending items that are waiting to be drawn
	plot_queue=[]
	## 	Function run_loop
	#	This is the main loop of this ROS Node. It continuously processes incoming QT events (draw requests).
	#	It replaces the standard ROS spin function. We can only do this in Python, since rospy's spin is just a sleep.
	def run_loop(self):
		# Spawn the graph window
		self.win = CustomGraphicsWindow()
		self.win.setWindowTitle('Plot Tool')
		self.graph = self.win.addPlot()
		self.graph.showGrid(x=True, y=True)
		# This needs to be called to process QT events (render the window)
		QtGui.QApplication.instance().processEvents()
		# Main run loop while ROS is still going
		while not rospy.is_shutdown():
			# Check pending queue for new plot items
			while len(self.plot_queue) > 0:
				p = self.plot_queue.pop(0)
				self.plot(p.x_set, p.y_set, p.series, p.append, p.symbol, p.symbol_size)
				# This renders each plot item as soon as it is added
				# This causes a slow down if items are plotted at a high frequency (50Hz)
				# However, this approach does not miss any plot items
				# If we don't care about missing plot items, we can remove this
				QtGui.QApplication.instance().processEvents()
			#Process QT events so graph can still be manipulated when there are no plot requests
			QtGui.QApplication.instance().processEvents()
	##	Function plot
	#	Takes Plot_Info items from the pending queue and creates a plot object that can be added to the graph
	#	@param x_set the list of x coordinates to be plotted
	#	@param y_set the list of y coordinates to be plotted
	#	@param series the series that the data belongs to
	#	@param append True: plot will be added to existing plots in the series. False: plot will replace all previous plots in the series.
	#	@param symbol Marker types: '-' for line plot, 'o' for circle, 's' for square, 't' for triangle, 'd' for diamond, '+' for cross
	#	@param symbol_size how big the markers will be (does not affect line plots)
	def plot(self,x_set,y_set,series,append,symbol,symbol_size):
		# Default settings will be applied if lacking information
		if series==None:
			rospy.logwarn("plot_tool: series not declared.. setting series to 0")
			series = 0
		elif series < 0 or series > 6:
			rospy.logwarn("plot_tool: series must be between 0 and 6.. setting series to 0")
			series = 0
		if append==None:
			rospy.logwarn("plot_tool: append not declared.. setting append to true")
			append=True
		if len(x_set) > 0 and len(y_set)>0:
			if symbol==None:
				rospy.logwarn("plot_tool: symbol not declared.. setting symbol to -")
				symbol='-'
			else:
				symbol=str(unichr(symbol))
				if symbol!='o' and symbol!='s' and symbol!='t' and symbol!='d' and symbol!='+' and symbol!='-':
					rospy.logwarn("plot_tool: symbol is not valid, should be one of { -,o,s,t,d,+ }.. setting symbol to -")
					symbol='-'
			if symbol_size==None:
				rospy.logwarn("plot_tool: symbol_size not declared.. setting symbol_size to 10")
				symbol_size=10
		# set plot color based on data series
		c = self.colors[series]	
		# clear existing plots if append is False
		if append==False:
			plot_list = self.plot_tracker[series]
			for plot in plot_list:
				plot.clear()
			self.plot_tracker[series]=[]
		# note that we do not always need data, one way to clear plots is to send a request with append = False and no data
		if x_set!=None and y_set!=None: 
			# line plots with 1 data point is automatically converted to a scatter plot
			if (len(x_set) == 1 or len(y_set) == 1) and symbol=='-':
				rospy.logwarn("plot_tool: can't plot line with single point.. changing symbol to o")
				symbol='o'
			if symbol=='-':
				plot = self.graph.plot(x_set,y_set, pen=c)
			else:
				plot = self.graph.plot(x_set,y_set, pen=None, symbolPen=c, symbolBrush=c, symbolSize=symbol_size, symbol=symbol)
			# save the plot, so we can access it for redrawing or deleting later
			self.plot_tracker[series].append(plot)

##	Class Plot_Info
#	This class is just used to organize the plot request parameters into a tidy object
class Plot_Info:
	def __init__(self, x_set, y_set, series, append, symbol, symbol_size):
		self.x_set = x_set
		self.y_set = y_set
		self.series = series
		self.append = append
		self.symbol = symbol
		self.symbol_size = symbol_size

## 	Function srv_plot_path
#	This is the service handler for plotting nav_msgs::Path messages.
#	@param req this is the standard service request object in ROS
def srv_plot_path(req):
	# Extract the plot data out of the nav_msgs::Path message 
	# and organize it into a list of x coordinates and a list of y coordinates
	data = req.msg
	if data==None:
		x_set=None
		y_set=None
	else:
		pose_list = data.poses
		N=len(pose_list)
		i=0
		x_set=[]
		y_set=[]
		while i<N:
			if (pose_list[i].pose.position.x != None and pose_list[i].pose.position.y != None):
				x_set.append(pose_list[i].pose.position.x)
				y_set.append(pose_list[i].pose.position.y)
			i=i+1
	# Create a Plot_Info object with request parameters and add it to the pending queue (for drawing)
	graph_obj.plot_queue.append(Plot_Info(x_set,y_set,req.series,req.append,req.symbol,req.symbol_size))
	# Return acknowledgement that the service is complete
	return PlotPathResponse()

## 	Function srv_plot_pose
#	This is the service handler for plotting geometry_msgs::Pose messages.
#	@param req this is the standard service request object in ROS
def srv_plot_pose(req):
	# Extract the plot data out of the geometry_msgs::Pose message.
	# and organize it into a list of x coordinates and a list of y coordinates
	data = req.msg
	if data==None:
		x_set=None
		y_set=None
	else:
		if (data.position.x != None and data.position.y != None):
			x_set=[data.position.x]
			y_set=[data.position.y]
	# Create a Plot_Info object with request parameters and add it to the pending queue (for drawing)
	graph_obj.plot_queue.append(Plot_Info(x_set,y_set,req.series,req.append,req.symbol,req.symbol_size))
	# Return acknowledgement that the service is complete
	return PlotPoseResponse()
            
##	Function node_setup
#	This function sets up the ROS Node and registers the ROS services it provides
def node_setup():
	# Create a global Graph_Drawer instance for everything to use/draw on
	# This is necessary because QTCore and QTGui does not like to be controlled from multiple threads
	# However, rospy spawns different threads to handle service requests and callbacks
	# To avoid rendering issues, we are using the queue system in Graph_Drawer for plot requests
	global graph_obj
	graph_obj = Graph_Drawer()
	# Start the ROS Node and register the services offered
	rospy.init_node('plot_path')
	drawLine_srv = rospy.Service("plot_tool/draw_path", PlotPath, srv_plot_path)
	drawPoint_srv = rospy.Service("plot_tool/draw_pose", PlotPose, srv_plot_pose)
	# Kick off the main run loop in the Graph_Drawer instance
	graph_obj.run_loop()

# 	Point of entry into the program
if __name__ == '__main__':
	node_setup()



