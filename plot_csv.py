#!/usr/bin/env python3

from nav_conversion import NavConversion
from helper_functions import *

# change Matplotlib's default font size
plt.rcParams['font.size'] = '14'

date = "17th of May"
duration = "10 min"

# specify origin for conversion between lat/lon and xy
navcon = NavConversion(42.2550228836434, -85.6376864496628, 252.000822331499)

# Lan/Lon points of Trimble (Ground Truth)
pts = np.array([ [-85.6376864229665,42.2550229157656,251.991689730179],
                [-85.6376864816832,42.2550228836929,252.008549407799],
                [-85.6376864858191,42.2550228929578,252.001319308508],
                [-85.6376864496628,42.2550228836434,252.000822331499] ])

def compute_accuracy(df, x, y, name, plot1, plot2, plot_sats):
    '''
        Plot GPS data from pandas dataframe by time and satellite numbers.

        params:
            - df: pandas dataframe
            - x: string of column name for x axis
            - y: string of column name for y axis
            - name: Title of plot
            - plot1: plot number for subplot
            - plot2: plot number for subplot
            - plot_sats: boolean. Whether to make the satellite number subplot.
    '''
    # mean of x and y data. Currently not being used.
    xmean = np.mean( df[x] )
    ymean = np.mean( df[y] )

    # horizontal and vertical accuracy published by the GPS sensor
    haccuracy = df['haccuracy'].max()/1000
    vaccuracy = df['vaccuracy'].max()/1000

    # maximum distance from both reported accuracy
    gaccuracy = np.sqrt( haccuracy**2 + vaccuracy**2 )

    xdist = df[x]
    ydist = df[y]

    d = np.sqrt( np.power(xdist, 2) + np.power(ydist, 2 ) )

    # prints the a
    # r1, r2, r3, r4 = 0.01, 0.02, 0.04, 0.06
    # print( "{:.2f}% fell under {} cm, {:.2f}% fell under {} cm, {:.2f}% fell under {} cm, {:.2f}% fell under {} cm".format(
    #     accuracy(d, r1), r1*100, accuracy(d, r2), r2*100, accuracy(d, r3), r3*100, accuracy(d, r4), r4*100 ) )

    # Start of subplot
    plt.subplot(plot1)
    plt.suptitle("GPS accuracy validation", fontweight='bold')

    plt.scatter( xdist, ydist, c=np.arange(len(xdist))/len(xdist) )
    cbar = plt.colorbar()
    plt.clim(0, 1)
    cbar.ax.set_ylabel("Normalized Time", fontweight='bold', rotation=270, labelpad=20.0)

    # draw horizontal and vertical line
    plt.axhline(0.0, color = 'r', linestyle = '-')
    plt.axvline(0.0, color = 'r', linestyle = '-')

    # # plot points from Trimble
    st = False
    for j in range(len(pts[:, 0])):
        xc, yc, zc = navcon.geodetic_to_enu(pts[j, 1], pts[j, 0], pts[j, 2])
        if not st:
            plt.scatter(xc, yc, c='r', label="Ground Truth")
            st = True
        else:
            plt.scatter(xc, yc, c='r')

    # draw circles of accuracy
    radius = gaccuracy
    cx, cy = create_circle(gaccuracy)
    plt.plot( cx, cy, color='m', label=str(format(radius*100, '.2f')) + " cm")

    # plot radius accuracy of specified r1, r2 and r3

    # radius = r1
    # cx, cy = create_circle(radius)
    # plt.plot( cx, cy, color='m', label=str(radius*100) + " cm")

    # radius = r2
    # cx, cy = create_circle(radius)
    # plt.plot( cx, cy, color='g', label=str(radius*100) + " cm")

    # radius = r3
    # cx, cy = create_circle(radius)
    # plt.plot( cx, cy, color='c', label=str(radius*100) + " cm")

    plt.xlabel('x (m)', fontweight='bold')
    plt.ylabel('y (m)', fontweight='bold')
    #plt.ylim(-1.0, 1.0)
    plt.title(date + " / " + duration + " - " + name, fontweight='bold')
    plt.grid()

    plt.legend(loc='upper left')

    if plot_sats:
        '''
            Only create this subplot if the csv file has satellite number
            data
        '''

        plt.subplot(plot2)

        d, sats = satellites_plots(df, x, y)

        for i in sats:
            plt.scatter(d[str(i)]['x'], d[str(i)]['y'], label=str(i))

        plt.grid()
        plt.xlabel('x (m)', fontweight='bold')
        plt.ylabel('y (m)', fontweight='bold')
        plt.title("Satellites number", fontweight='bold')

        plt.legend(loc='upper left')

    # plot confidense ellipse for standard deviations
    confidence_ellipse(df[x], df[y], n_std = [1.0, 2.0, 3.0], linewidth=3)

def main():

    # read csv file
    df = pd.read_csv('05-17-corrections.csv')

    compute_accuracy( df , 'x', 'y', "Corrections", 121, 122, False)

    # compute_accuracy( pd.read_csv('05-17-ncorrections.csv') , 'x', 'y', "Corrections", 121, 122)
    # compute_accuracy( pd.read_csv('05-17-corrections.csv') , 'x', 'y', "IMU & GPS Fusion", 121, 122)
    # compute_accuracy( pd.read_csv('05-17-loosely_coupled.csv') , 'y', 'x', "IMU & GPS & Odom Fusion", 121, 122)

    # plt.subplots_adjust(hspace=0.5)

    plt.tight_layout()
    plt.show()

try:
    main()
except KeyboardInterrupt:
    pass