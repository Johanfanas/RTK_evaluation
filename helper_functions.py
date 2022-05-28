#!/usr/bin/env python3

from os import remove
import random
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors as mcolors
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

def create_circle(r):
    '''
        Creates a circle of a radius R.
    '''
    theta = np.linspace( 0 , 2 * np.pi , 150 )
 
    radius = r
    
    x = radius * np.cos( theta )
    y = radius * np.sin( theta )

    return x, y

def accuracy(d, precision):
    '''
        This function assumes d is a pd.Series; hence
        np.where will return a tuple and we extract the
        first term of tuple.
    '''
    return len( np.where(d < precision)[0] )/len(d)*100

def satellites_plots(df, x, y):
    '''
        Plot data with different color depending on 
        satellite number

        params:
            - df: pandas dataframe
            - x: string of column name for x axis
            - y: string of column name for y axis
        
        returns:
            - d: dictionary of raw data and satellite number
            - sats: unique satellite numbers
    '''
    xmean = np.mean( df[x] )
    ymean = np.mean( df[y] )

    df[x] = df[x]
    df[y] = df[y]

    d = {}

    sats = df['sats'].unique()

    for j in sats:
        d[str(j)] = df[ df['sats'] == j ]
        print( "{} number of points for {} satellites".format( d[str(j)][x].count(), j ) )

    return d, sats

def cep(df, x, y):
    xmean = np.mean( df[x] )
    ymean = np.mean( df[y] )

    xdist = df[x] - xmean
    ydist = df[y] - ymean

    print( xdist.sum()/len(xdist) )
    print( ydist.sum()/len(ydist) )

    d = np.sqrt( np.power(xdist, 2) + np.power(ydist, 2 ) )
    angles = np.arctan2( ydist, xdist )
    xd = np.cos(angles)*d
    yd = np.sin(angles)*d

    sigma_dx = xd.std()
    sigma_dy = yd.std()

    radius_cep = 0.56*sigma_dx + 0.62*sigma_dy

    print( radius_cep )
    print( accuracy(d, 0.2) )

    plt.scatter( xdist, ydist )
    #plt.gca().add_patch( plt.Circle( (0,0), radius_cep, color='r' ) )
    plt.grid()
    plt.show()

def confidence_ellipse(x, y, n_std=[3.0], facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of `x` and `y`

    Parameters
    ----------
    x, y : array_like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : array of standard deviations
        The number of standard deviations to determine the ellipse's radiuses.

    Returns
    -------
    matplotlib.patches.Ellipse

    Other parameters
    ----------------
    kwargs : `~matplotlib.patches.Patch` properties
    """

    # fig, ax = plt.subplots(1, 1, figsize=(9, 3))
    fig, ax = plt.subplots(1, 1)

    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    color_list = list( mcolors.BASE_COLORS.keys() )
    color_list.remove('w')

    print("---------------------------")

    # plot multiple ellipes with a standard deviation
    for std, color in zip(n_std, random.sample(color_list, 3) ):

        cov = np.cov(x, y)
        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensionl dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0),
            width=ell_radius_x * 2,
            height=ell_radius_y * 2,
            facecolor=facecolor,
            label=str(std) + " \u03C3",
            edgecolor=color,
            **kwargs)
        
        # Calculating the stdandard deviation of x from
        # the squareroot of the variance and multiplying
        # with the given number of standard deviations.
        scale_x = np.sqrt(cov[0, 0]) * std
        mean_x = np.mean(x)

        # calculating the stdandard deviation of y ...
        scale_y = np.sqrt(cov[1, 1]) * std
        mean_y = np.mean(y)

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + ax.transData)
        ax.add_patch(ellipse)

        print("Confidence std in X {} and std in Y {} for {} \u03C3".format(scale_x, scale_y, std))
    
    ax.scatter( x, y )
    ax.axvline(x=mean_x, c='k', lw=1)
    ax.axhline(y=mean_y, c='k', lw=1)
    ax.set_xlabel('x (m)', fontweight='bold')
    ax.set_ylabel('y (m)', fontweight='bold')
    ax.set_title("Confidence Ellipse", fontweight='bold')
    ax.grid()
    ax.legend(loc='upper left')