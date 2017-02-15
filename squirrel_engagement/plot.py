import plotly
import os.path
import sys

from plotly.graph_objs import Scatter, Layout

def plot_ly(times, levels):
	print plotly.__version__  # version >1.9.4 required	
	plotly.offline.plot({
	"data": [
    	Scatter(x = times, y = levels)
	],
	"layout": Layout(
	    title="participation level"
	)
	})

def plot_ly(times, level_1, level_2, level_3, name, y_name):
	print plotly.__version__  # version >1.9.4 required	

	level_1 = Scatter(x = times, y = level_1, name = 'speaker 1')
	level_2 = Scatter(x = times, y = level_2, name = 'speaker 2')
	level_3 = Scatter(x = times, y = level_3, name = 'speaker 3')

	m_data = [level_1, level_2, level_3]

	m_layout = dict(title = name,
              xaxis = dict(title = "time (sec)"),
              yaxis = dict(title = y_name),
              )
	fig = dict(data=m_data, layout = m_layout)
	plotly.offline.plot(fig, filename = (name + '.html'))
	

if len(sys.argv) != 6:
	print('usage: predictions_file_0 predictions_file_1 predictions_file_2')
	quit()

predictions_file_0 = open(sys.argv[1])
predictions_file_1 = open(sys.argv[2])
predictions_file_2 = open(sys.argv[3])
pos = int(sys.argv[4])
name = sys.argv[5]

times_0 = []
levels_0 = []
times_1 = []
levels_1 = []
times_2 = []
levels_2 = []

time = 0
window_size = 5
for line in predictions_file_0:
	if(time == 0):
		time = time + window_size
		continue
	levels_0.append(float(line.split("\t")[pos]))
	times_0.append(time)
	time = time + window_size

time = 0
for line in predictions_file_1:
	if(time == 0):
		time = time + window_size
		continue
	levels_1.append(float(line.split("\t")[pos]))
	times_1.append(time)
	time = time + window_size

time = 0
for line in predictions_file_2:
	if(time == 0):
		time = time + window_size
		continue
	levels_2.append(float(line.split("\t")[pos]))
	times_2.append(time)
	time = time + window_size
print "ready to plot"
plot_ly(times_0, levels_0, levels_1, levels_2, name, name)
