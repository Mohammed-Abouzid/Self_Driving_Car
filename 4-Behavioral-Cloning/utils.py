import csv
import numpy as np
import cv2
import matplotlib.image as mpimg
from sklearn.utils import shuffle


# load the dataset
def load_data(images, measurements, csv_name, img_dir):
    path = './data/' + csv_name + '.csv'
    lines = []
    with open(path) as csvfile:    # change this to the data path
        reader = csv.reader(csvfile)
        for line in reader:
            lines.append(line)
    lines.pop(0) # remove the first line, as it's not data but titels.

    for line in lines:
        for i in range(3):    # load the center, left and right data
            f_name = line[i].split('/')[-1]
            curr_path = './data/' + img_dir + '/' + f_name
            image = cv2.cvtColor(cv2.imread(curr_path), cv2.COLOR_BGR2RGB)
            images.append(np.array(image))
            
        measur_center= float(line[3])  # steering measurement for the center images.
        measur_left  = measur_center + 0.4  # steering measurement for left
        measur_right = measur_center - 0.4  # steering measurement for right
        measurements.append(measur_center)
        measurements.append(measur_left)
        measurements.append(measur_right)
             
        
        
        
     
def generator(data, batch_s=64, type_= 'x' ):
    data_len = len(data)
    while 1: 
        for offset in range(0, data_len, batch_s):          #batch_size = 64
            batch = data[offset:int(offset+batch_s)]  #batch_size = 64

            images = []
            measurement = []
            for b in batch:
                #print('---', b)
                #print('+++', b[0])
                if type_=='x':
                    name = '../data/'+b[0]
                else:
                    f_name = b[0].split('\\')[-1]
                    img_dir = b[0].split('\\')[-2]
                    name = './data/' + img_dir + '/' + f_name
                image = mpimg.imread(name)
                steer = float(b[3])
                images.append(image)
                measurement.append(steer)

            X_train = np.array(images)
            y_train = np.array(measurement)
            
            yield shuffle(X_train, y_train)