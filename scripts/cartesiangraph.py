import numpy as np
import matplotlib.pyplot as plt
import pandas

###############################################################################
def build_cartesian_plane(max_quadrant_range):
    #The quadrant range controls the range of the quadrants
    plt.grid(True, color='b', zorder=0,)
    ax = plt.axes()
    head_width = float(0.05) * max_quadrant_range
    head_length = float(0.1) * max_quadrant_range
    ax.arrow(0, 0, max_quadrant_range, 0, head_width=head_width, head_length=head_length, fc='k', ec='k',zorder=100)
    ax.arrow(0, 0, -max_quadrant_range, 0, head_width=head_width, head_length=head_length, fc='k', ec='k', zorder=100)
    ax.arrow(0, 0, 0, max_quadrant_range, head_width=head_width, head_length=head_length, fc='k', ec='k', zorder=100)
    ax.arrow(0, 0, 0, -max_quadrant_range, head_width=head_width, head_length=head_length, fc='k', ec='k', zorder=100)
    
    # adds dashes to plot
    '''l = []
    zeros = []
    counter_dash_width = max_quadrant_range * 0.02
    dividers = [0,.1,.2,.3,.4, .5, .6, .7, .8, .9, 1]
    for i in dividers:
        plt.plot([-counter_dash_width, counter_dash_width], [i*max_quadrant_range, i*max_quadrant_range], color='k')
        plt.plot([i * max_quadrant_range, i*max_quadrant_range], [-counter_dash_width, counter_dash_width], color='k')
        plt.plot([-counter_dash_width, counter_dash_width], [-i * max_quadrant_range, -i * max_quadrant_range], color='k')
        plt.plot([-i * max_quadrant_range, -i * max_quadrant_range], [-counter_dash_width, counter_dash_width], color='k')
        l.append(i * max_quadrant_range)
        l.append(-i * max_quadrant_range)
        zeros.append(0)
        zeros.append(0)'''

###############################################################################
def cartesian_spines():
    ax = plt.gca()
    ax.set_title('yo')
    ax.spines['top'].set_color('none')
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')
    ax.spines['right'].set_color('none')

    graph_range = 10
    ax.arrow(0, 0, 2, 4, head_width=float(.05)*graph_range,
             head_length=float(.1)*graph_range, 
             fc='k', ec='k',zorder=100)
    #x = np.linspace(-np.pi, np.pi, 100)
    x  = [0,-2]
    #y = np.sin(x)
    y=[0,-4]
    ax.plot(x, y)
    ax.plot([-graph_range],[graph_range])

###############################################################################
cartesian_spines()
#build_cartesian_plane(10)
plt.show()
