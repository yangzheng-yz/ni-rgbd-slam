#!/usr/bin/python

# Copyright (c) <2016>, <Nanyang Technological University> All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
from matplotlib.lines import lineStyles
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy
from numpy import *
from statsmodels.distributions.empirical_distribution import ECDF
from tf import transformations 

filename = "/home/eee/drones/src/ni_slam/script/tum/fr1_desk_est.txt"

if __name__ == "__main__":

	if len(sys.argv) > 1 :
		filename = sys.argv[1]
	else:
		print "Using default filename", filename

	file = open(filename, 'r')
	file.readline()
	results = array([map(float, line.split(' ')) for line in file if line.strip() !=""])
	pos_est = results[:,1:4]
	pos_rel = results[:,8:11]
	rot_est = results[:,4:8]
	rot_rel = results[:,11:15]
	max_res = results[:,15]
	psr     = results[:,16]
	time = results[:,0] - results[0,0]

	# t = 23
	fig = plt.figure()

	ax = fig.add_subplot(411)
	ax.plot(time, pos_est, marker=".")
	
	ax.plot(time, pos_rel)
	plt.legend(['est x', 'est y', 'est z','real x', 'real y', 'real z'])
	# plt.xlim((0,t))
	plt.grid()

	ax = fig.add_subplot(412)
	plt.plot(time,rot_est,marker=".")
	plt.plot(time,rot_rel)
	plt.legend(['est q', 'real q'])
	# plt.xlim((0,t))
	plt.grid()

	ax = fig.add_subplot(413)
	euler_est = array([transformations.euler_from_quaternion(line) for line in rot_est])
	euler_rel = array([transformations.euler_from_quaternion(line) for line in rot_rel])
	plt.plot(time,euler_est/pi*180,marker=".")
	plt.plot(time,euler_rel/pi*180)
	plt.legend(['est roll', 'real roll','est pitch', 'real pitch','est yaw', 'real yaw'])
	# plt.xlim((0,t))
	plt.grid()

	ax = fig.add_subplot(414)
	plt.plot(time,psr)
	plt.legend(['PSR'])
	# plt.xlim((0,t))
	plt.grid()

	fig = plt.figure()
	ax = fig.add_subplot(211)
	error = abs(pos_est - pos_rel)
	plt.plot(time,error)
	plt.legend(['error x', 'error y', 'error z','error p'])


	ax = fig.add_subplot(212)
	e = linspace(0,error.max(),1000).reshape(1000)
	pos_error = sqrt(error[:,0]**2 + error[:,1]**2+error[:,2]**2)
	ecdf_x = ECDF(error[:,0])([e])[0]
	ecdf_y = ECDF(error[:,1])([e])[0]
	ecdf_z = ECDF(error[:,2])([e])[0]
	ecdf_p = ECDF(pos_error)([e])[0]
	
	plt.plot(e, ecdf_x, marker=".")
	plt.plot(e, ecdf_y, marker=".")
	plt.plot(e, ecdf_z, marker=".")
	plt.plot(e, ecdf_p, marker=".")
	plt.legend(['ecdf x', 'ecdf y', 'ecdf z','ecdf p'])
	plt.xlabel('Error/m')
	plt.grid()

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(pos_est[:,0], pos_est[:,1], pos_est[:,2])
	ax.plot(pos_rel[:,0], pos_rel[:,1], pos_rel[:,2])

	path_lenth = sum(array([sqrt(sum((pos_rel[i+1]-pos_rel[i])**2)) for i in xrange(0, pos_rel.shape[0]-1)]))
	print "path lenth:    ", path_lenth
	print "abs mean error:", mean(error,0)
	print "relative drift:", mean(error,0)/path_lenth*100, "%"
	print "RMSE drift:    ", mean(sqrt(sum(error**2,1)))
	print "rel RMSE drift:", mean(sqrt(sum(error**2,1)))/path_lenth*100,"%"#/path_lenth*100, "%"
	print "Frequency:     ", time.shape[0]/time[-1]
	
	plt.show()
