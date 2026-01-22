#!/usr/bin/env python3
#
# Compare colors of two different items using a SVM
#
# When used with pictures of apple and pears it was found :- 
# Blue-Red is not classified because the data points are mixed. In comparison, if you look at the blue-green and green-red diagrams, you can clearly separate apples from pears.
# From this, it was found that green information is important to distinguish between pears and apples, not red or blue.
# In the RGB image, the apple is based on Red, but the pear is yellow-green, so it is based on Green and Red.
# So, there was a clear difference in green color between apples and pears.
#
import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from sklearn.svm import LinearSVC
import mglearn

# make the bgr analysis files in csv format for input to the SVM 
odir = 'output/'
fname1='apple'                                                                 # the first item pictures folder
fname2='pear'                                                                  # the second item pictures folder
fname_list = [ fname1, fname2 ]
num_photo=0
for x in os.listdir(fname1):
    y = x.lower()
    if y.endswith(".jpg") or y.endswith(".jpeg") or y.endswith(".png"):
        num_photo += 1
num_photo2=0
for x in os.listdir(fname2):
    y = x.lower()
    if y.endswith(".jpg") or y.endswith(".jpeg") or y.endswith(".png"):
        num_photo2 += 1
num_photo=min(num_photo, num_photo2)                                            # use number of photos that are same so take min
bgr = np.zeros((num_photo,3))                                                   # init storage array for each photo
for fname in fname_list:
    for n, x in enumerate(os.listdir(fname)):
        y = x.lower()
        if y.endswith(".jpg") or y.endswith(".jpeg") or y.endswith(".png"):
            print(x)
            img = cv2.imread(x)                                                  # read the picture file  
            h, w, c = img.shape                                                  # height, width, channnel
            l=0
            b_ave=0; g_ave=0; r_ave=0
            for i in range(h):
                for j in range(w):
                    if(img[i,j,0] != 0 or img[i,j,1] != 0 or img[i,j,2] != 0 ):   # [0,0,0]（Black）
                        l+=1   
                        b_ave=b_ave+img[i,j,0]
                        g_ave=g_ave+img[i,j,1]
                        r_ave=r_ave+img[i,j,2]
            b_ave=b_ave/l
            g_ave=g_ave/l
            r_ave=r_ave/l
            bgr[k]=np.array([b_ave, g_ave, r_ave])
            if (n+1) == num_photo:                                               # break when all pictures have been read 
                break
    df = pd.DataFrame(bgr, columns=['blue', 'green', 'red'])    
    df.to_csv(odir + fname + '.csv')

# run SVM on each color comparison
def main():
    path = 'output'
    os.makedirs(path, exist_ok=True)
    first = np.loadtxt(odir + fname1 + '.csv', delimiter=',', skiprows=1,usecols=[1,2,3])   # read each input file
    secnd = np.loadtxt(odir + fname2 + '.csv', delimiter=',', skiprows=1,usecols=[1,2,3])
    SVM2D(np.delete(first,2,1), np.delete(secnd,2,1),'blue','green',path)                   # bgr blue-green SVM
    SVM2D(np.delete(first,0,1), np.delete(secnd,0,1), 'green', 'red',path)                  # bgr greem-red SVM
    SVM2D(np.delete(first,1,1), np.delete(secnd,1,1), 'blue', 'red',path)                   # bgr blue-red SVM

def SVM2D(ap_pv, pe_pv, xlabel, ylabel, path):

    yap=[0]*ap_pv.shape[0]  
    ype=[1]*pe_pv.shape[0]  
    y = np.array(yap+ype)  
    X = np.concatenate([ap_pv,pe_pv],0)    

    # SVM Model
    linear_svm = LinearSVC().fit(X, y)
    fig=plt.figure(figsize = (10, 6))
    ax = fig.add_subplot(1,1,1)
    ax.axis('normal')
    mglearn.plots.plot_2d_separator(linear_svm, ap_pv)
    mglearn.discrete_scatter(X[:, 0], X[:, 1], y)

    ax.legend([fname1, fname2])
    ax.xaxis.set_major_locator(ticker.MultipleLocator(20))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(20))
    ax.tick_params('x', labelsize =15)
    ax.tick_params('y', labelsize =15)
    ax.set_xlabel(xlabel, fontsize= 20)
    ax.set_ylabel(ylabel, fontsize= 20)
    plt.savefig(path+'/SVM_'+xlabel+'_'+ylabel+'.png')

    print('score on training set: {:.2f}'.format(linear_svm.score(X,y)))

if __name__=='__main__':
    main()