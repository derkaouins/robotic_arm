# import the necessary packages
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
import cv2
from sklearn.cluster import KMeans

class ColorLabeler:

	cnts = None
	kmeans = None
	
	def __init__(self,contours,n):
		global cnts
		global kmeans
		cnts = contours
		kmeans = KMeans(n_clusters=n)
		means = [row[1] for row in cnts]
		kmeans.fit(means)
		
	def getMostColor(self, mean):
		
		blue = mean[0]
		green = mean[1]
		red = mean[2]
		
		maxD = max(red, green, blue)
		if red == maxD:
			return 0 #"red"
		if green == maxD:
			return 1 #"green"
		if blue == maxD:
			return 2 #"blue"
			
	def getLabel(self,cnts,supervised=False):
		means = [row[1] for row in cnts]
		if len(means) == 0:
			return []
			
		if supervised :
			tmp = []
			for mean in means:
				tmp.append(self.getMostColor(mean))
			return tmp
		
		global kmeans
		return kmeans.predict(means)
			
	def getCenters(self, ctrs):
		#means = [row[1] for row in ctrs]
		#kmeans = KMeans(n_clusters=3)
		#kmeans.fit(means)
		#print("\n\n")
		#print(kmeans.cluster_centers_)
		
		i = 0
		rows, cols = (len(ctrs), 3) 
		tmp = [[0]*cols]*rows
		for c in ctrs:
			tmp[i] = [c[0],c[1],kmeans.labels_[i]]
			i += 1	
		
		return tmp
		
