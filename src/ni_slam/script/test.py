#!/usr/bin/python
#import arrayfire as af
import time
import numpy as np
import random
import cv2
class jtimer:
    def __init__(self):
        self.start_time = time.time()

    def start(self):
        self.start_time = time.time()

    def end(self):
        self.time_use = time.time()-self.start_time
        self.start_time = time.time()
        return self.time_use

    def tic(self):
    	self.start()

    def toc(self, name=""):
    	self.end()
    	print name, self.time_use, 's', 1/self.time_use, 'Hz'

timer = jtimer()
	



A = np.random.rand(500,500)
print A.shape
for x in xrange(100):
	timer.tic()
	B1 = np.fft.fft2(A)
	timer.toc("np:")
	
	timer.tic()
	B2 = cv2.dft(A, flags = cv2.DFT_COMPLEX_OUTPUT)
	C = B2[:,:,0]+1j*B2[:,:,1]
	timer.toc("cv:")

print np.linalg.norm(B1-C)

a = np.random.rand(360,240)
timer.tic()
print np.linalg.norm(a)
timer.toc("norm")
print np.sum(a)
timer.toc("sum ")
# print sum_a.shape, sum_a, 