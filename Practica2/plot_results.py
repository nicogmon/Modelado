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


files = ['Fase3_nicolas_garcia.csv'] # Files to be included in the plot
def plot_results():
    # Read the csv file
    fig, ax = plt.subplots()
    legend = []
    for file in files:
        with open(file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            
            
            G_parcial = []
            time = []
            for row in reader:
                
                G_parcial.append(float(row[2]))
                time.append(float(row[0]))

                
                
            std = round(np.std(G_parcial),3)
            G_total = round(np.sum(G_parcial),3)
            
            
            G_parcial_arr = np.asarray(G_parcial)
            time_arr = np.asarray(time)
            
            ax.plot(time_arr, G_parcial_arr) 

            #ax.plot(dist_arr, angles)
        
            
    ax.set(xlabel='Time (s)', ylabel='Force (N)',
               title='tiempo vs G-parcial\nG-parcial_dst = ' + str(std)+ ' N '+ 'G-total = ' + str(G_total) + ' N')
    ax.grid()
    
    ax.legend(legend)
    
    
    plt.show()
        
        
        
        

        
def main():
    while True:
        print("Waiting for measures.csv")
        plot_results()
        exit(0)
        
        

main()
