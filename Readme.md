## Overview and motivation:

This ROS package provides the ability to plot parametric 2D plots (i.e. an X, Y plot). It uses pyqtgraph as the back-end plotting library, and currently supports plotting of nav_msgs::Path and geometry_msgs::Pose.

Parametric plotting is currently not provided by existing ROS plotting packages rxplot and rqt_plot, as both of these plot against time. While RViz can handle parametric plots, this is a much lighter and more streamlined tool for when you just want to quickly check a robot path.

## Installation:

1. Download and install pyqtgraph from: http://www.pyqtgraph.org/ (current version is v0.9.8).
2. Grab the ROS package from this repository.
3. Copy the ROS packages to your catkin workspace, and run catkin_make.

## Running:

The plotting tool can be brought up with:

- **rosrun plot_tool plot_path.py**

The demo can be brought up with:

- **rosrun plot_demo demo**

## Features and usage:

The plot_tool offers two ROS services:

- **/plot_tool/draw_path** (for plotting nav_msgs::Path messages)
- **/plot_tool/draw_pose** (for plotting geometry_msgs::Pose messages)

The ROS srv structures are (respectively):

- PlotPath.srv (#include "plot_tool/PlotPath.h")
- PlotPose.srv (#include "plot_tool/PlotPose.h")

Both services have the following request fields (the only difference is in the msg type depending on the service):

| Field | Type | Descriptions |
--------|------|--------------|
|msg	| nav\_msgs::Path  geometry\_msgs::Pose | The message containing the position data to plot. |
|series	| uint32 | Select a series for the data to belong to. Data of the same series will be grouped together, and have its own color. Valid values lie between 0 and 6 (inclusive). See color table below for series/color correspondence. |
|append	| bool | True: the data will be plotted, and all existing plots in the series will be kept. False: the data will be plotted, and all existing plots in the same series will be cleared. |
|symbol	| char | The type of plot marker to use. See symbol table below for valid values. |
|symbol_size | uint32 | The size of the plot markers (has no affect if line plot is selected). |

|Series|Color|
|------|--------|
|0	 |  	green |
|1	 |  	red |
|2   |		blue |
|3	 |   	cyan |
|4	 |   	magenta |
|5	 |   	yellow |
|6	 |   	white |

|Symbol	| Marker Type |
|-------|-------------|
|-		|		Line graph (no point markers) |
|o		|		Scatter plot (dots as markers)|
|s		|		Scatter plot (squares as markers) |
|t		|		Scatter plot (triangles as markers) |
|d		|		Scatter plot (diamonds as markers) |
|+		|		Scatter plot (crosses as markers) |

Note that the service request does not have to populate all the fields. If any fields are missing, plot_tool will resort to a default value and give a ROS warning detailing the values actually used.

To clear a plot from the graph, simply send a request with append = false with either an empty msg field or attach a msg with no position data.

The services do not have any response fields.
