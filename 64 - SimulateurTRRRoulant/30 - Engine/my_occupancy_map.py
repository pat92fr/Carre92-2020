import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image

class occupancy_map:
    def __init__(self,size_x,size_y,scale): #,ax,ay,bx,by):
        self.carto = np.zeros((size_x,size_y), dtype=np.uint8)
        self.carto.fill(125)
        self.carto_offset_x = int(size_x/2.0)
        self.carto_offset_y = int(size_y/2.0)
        self.carto_offset = np.array([self.carto_offset_x,self.carto_offset_y])
        self.carto_scale = scale
        # self.ax = ax
        # self.ay = ay
        # self.bx = bx
        # self.by = by

    def load(self,filename):
        img = Image.open(filename)
        self.carto = np.array(img).T
        self.carto = cv2.GaussianBlur(self.carto,(5,5),0)
        self.carto = cv2.GaussianBlur(self.carto,(3,3),0)


    def plot(self):
        plt.imshow(self.carto)
        plt.show()

    def save(self):
        self.carto_blur = cv2.GaussianBlur(self.carto,(3,3),0)
        img = Image.fromarray(self.carto_blur, 'L')
        img.save('occupancy_map.png')

    def ray(self,origin,end,collision=True):
        rescaled_origin = origin*self.carto_scale+self.carto_offset
        rescaled_end = end*self.carto_scale+self.carto_offset
        cv2.line(
            self.carto,
            (int(rescaled_origin[1]),int(rescaled_origin[0])),
            (int(rescaled_end[1]),int(rescaled_end[0])),
            0, #color 0 = visited, no obstacle
            2)
        if collision:
            cv2.circle(
                self.carto,
                (int(rescaled_end[1]),int(rescaled_end[0])),
                3, 
                255,  #color 255 = visited, obstacle
                -1) # filled

    def valid_position(self,x,y):
        valid = self.carto[x,y] == 0
        return valid

    def test_ray(self,origin,end,):
        rescaled_origin = origin*self.carto_scale+self.carto_offset
        rescaled_end = end*self.carto_scale+self.carto_offset
        color = self.carto[int(rescaled_end[1]),int(rescaled_end[0])] 
        if color <= 128:
            return 0.0
        else:
            return (float(color)-128.0)/128.0

