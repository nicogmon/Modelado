#  Esta aplicación tendrá la
# funcionalidad de generar un gráfico que exhiba de manera clara y precisa la variación de la
# velocidad del robot en función de su posición. Específicamente, la posición del robot se
# mostrará en el eje X, mientras que su velocidad se representará en el eje Y. 


import matplotlib.pyplot as plt
import numpy as np

import matplotlib as mpl
import csv
import pandas as pd

import math
import os
import time

files = ['Fase3.csv','Fase4.csv'] # Files to be included in the plot
def plot_results():
    # Read the csv file
    fig, ax = plt.subplots()
    legend = []
    for file in files:
        with open(file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            
            
            dists = []
            vels = []
            forces = []
            vels_ad = []    
            vels_tr = []
            angles = []
            for row in reader:
                
                dists.append(float(row[1]))
                vels.append(float(row[2]))
                # forces.append(float(row[7]))
                # vels_ad.append(float(row[3]))
                # vels_tr.append(float(row[5]))
                angles.append(float(row[10])/10)

                
                
            MSE = np.square(np.subtract(vels, 2)).mean()
            RMSE = math.sqrt(MSE) #performance measure
            print( "Archivo: ", file.name, "RMSE : ", RMSE)
            
            dist_arr = np.asarray(dists)
            vel_arr = np.asarray(vels)
            
            ax.plot(dist_arr, vel_arr) 
            legend.append((file.name, "RMSE: " + str(RMSE))) #add to legend file name and RMSE
            #ax.plot(dist_arr, angles)
        
            
    ax.set(xlabel='Distance (m)', ylabel='Velocity (m/s)',
               title='Velocity vs Distance')
    ax.grid()
    
    ax.legend(legend)
    
    
    plt.show()
        
        
        
        

        
def main():
    while True:
        print("Waiting for measures.csv")
        plot_results()
        exit(0)
        
        

main()
