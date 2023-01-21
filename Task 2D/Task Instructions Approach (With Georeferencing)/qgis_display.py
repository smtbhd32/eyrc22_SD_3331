#!/usr/bin/env python3

import rospy
from sentinel_drone.msg import Geolocation
from qgis.utils import *
from qgis.core import *
from  gdalconst import *
from PyQt5.QtCore import QFileInfo,QSettings
from qgis.core import QgsRasterLayer, QgsCoordinateReferenceSystem

def loc_plotter(Geolocation):
    rospy.loginfo(Geolocation)
    canvas = iface.mapCanvas() 
    pnt = QgsPointXY(Geolocation.long, Geolocation.lat)
    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('255,0, 0'))
    m.setIconType(QgsVertexMarker.ICON_X)
    m.setIconSize(12)
    m.setPenWidth(5)
    m.setFillColor(QColor(0, 200, 0))

def Plot():
    file = "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif"
    fileInfo = QFileInfo(file)    
    baseName = fileInfo.baseName()
    layer = QgsRasterLayer(file, baseName)  
    QgsProject.instance().addMapLayer(layer)
    rospy.init_node('QGIS_Listener')
    rospy.Subscriber('/geolocation', Geolocation, loc_plotter)

Plot()




