# By Panstan
# Requirements: matplotlib and numpy (should be installed with matplotlib)
from datetime import datetime
from typing import Iterable
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from random import randrange
import matplotlib.dates as dates 
import numpy as np
# Threads aren't friendly with matplotlib in two threads, use multiprocessing instead
import multiprocessing
# This function plots the current and desired state
# Note: "pos" is position
def plot_current_desired_pos(desired_state, max_data_points=500, refresh_rate=10):
    # Define graph variables
    global time, current_pos, desired_pos
    time, current_pos, desired_pos = [], [], []
    # Main graph
    figure = pyplot.figure()
    # Format y axis as "Hour:Minute"
    figure.gca().axes.xaxis.set_major_formatter(dates.DateFormatter('%M:%S'))
    # Current pos line plot
    current_pos_line, = pyplot.plot_date(time, desired_pos, '-')
    # Desired pos line plot
    desired_pos_line, = pyplot.plot_date(time, desired_pos, '-')
    def update(frame):
        global time, current_pos, desired_pos
        time.append(datetime.now())
        # Adding current pos to current pos line plot
        current_sine_value = (desired_state + 1) / 2 + (desired_state - 1) / 2 * np.sin(len(time) * 0.1) + 25
        current_pos.append(current_sine_value)
        # Graph desired pos to desired pos line plot
        desired_pos.append(desired_state)
        # If the amount of listed datapoints on the x axis exceeds "max_data_points" reset graph
        if len(time) > max_data_points:
            time = []
            current_pos = []
            desired_pos = []
        # Graph current pos on line plot
        current_pos_line.set_data(time, current_pos)
        # Graph desired current pos on line plot
        desired_pos_line.set_data(time, desired_pos)
        # I have fucking idea what this does but it works
        figure.gca().relim()
        # Autoscale to remove excess data
        figure.gca().autoscale_view()
        # Send line plots to graph on main Graph 
        return current_pos_line, desired_pos_line
    # Render graph frames every x milliseconds (refresh_rate) on main Graph using update function
    animation = FuncAnimation(figure, update, interval=refresh_rate)
    pyplot.title("current and desired position")
    # Show plot
    pyplot.show()
# This function plots the current speed
def plot_speed(max_data_points=30, refresh_rate=10):
    # Define graph variables
    global time, speed
    time, speed = [], []
    # Main graph
    figure = pyplot.figure()
    # Format y axis as "Hour:Minute"
    figure.gca().axes.xaxis.set_major_formatter(dates.DateFormatter('%M:%S'))
    # Speed line plot
    speed_line, = pyplot.plot_date(time, speed, '-')
    def update(frame):
        global time, speed
        time.append(datetime.now())
        # Adding current speed to speed line plot
        speed.append(randrange(0, 100))
        # If the amount of listed datapoints on the x axis exceeds "max_data_points" start removing excess data from graph
        if len(time) > max_data_points:
            time = time[-max_data_points:]
            speed = speed[-max_data_points:]
        # Graph current speed on line plot
        speed_line.set_data(time, speed)
        # I have fucking idea what this does but it works
        figure.gca().relim()
        # Autoscale to remove excess data
        figure.gca().autoscale_view()
        # Send line plots to graph on main Graph 
        return speed_line
    # Render graph frames every x milliseconds (refresh_rate) on main Graph using update function
    animation = FuncAnimation(figure, update, interval=refresh_rate)
    pyplot.title("current speed")
    # Show plot
    pyplot.show()

if __name__ == "__main__":
    # Creating processes input 50 as desired pos for plot_current_desired_pos
    position_plot = multiprocessing.Process(target=plot_current_desired_pos, args=(50,))
    speed_plot = multiprocessing.Process(target=plot_speed)
  
    # Starting process plot_current_desired_pos
    position_plot.start()
    # Starting process speed_plot
    speed_plot.start()
  
    # Wait until process plot_current_desired_pos is finished
    position_plot.join()
    # Wait until process speed_plot is finished
    speed_plot.join()
