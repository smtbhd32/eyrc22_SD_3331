import os
from qgis.core import QgsVectorLayer
from tkinter import * 

#location of task2d.tiff file
file = "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif"

#Opening OpenStreetMap if it's not opened
sources = [layer.source() for layer in QgsProject.instance().mapLayers().values()]
source_found = False
for source in sources:
    if 'xyz&url' in source:
        source_found = True
        print('!!!Open Street Map Already Loaded!!!')
    if file in source:
        file_found = True
        print('!!!Task2D.tif Loaded!!!')
if not source_found:
    print('LOADING Open Street Map...')
    urlWithParams = 'type=xyz&url=http://a.tile.openstreetmap.org/%7Bz%7D/%7Bx%7D/%7By%7D.png&zmax=19&zmin=0&crs=EPSG4326'
    rlayer = QgsRasterLayer(urlWithParams, 'OpenStreetMap', 'wms')
    if rlayer.isValid():
        QgsProject.instance().addMapLayer(rlayer)
    else:
        print('invalid layer')

#Specifying the extent of Arena using task_2d.tiff file
fileInfo = QFileInfo(file)    
baseName = fileInfo.baseName()
layer = QgsRasterLayer(file, baseName)  

#Attaching the .csv file, that gets updated with location when a new block is found...
uri = r"file:///home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/plot.csv?type=csv&detectTypes=yes&xField={}&yField={}&crs=EPSG:4326&spatialIndex=no&subsetIndex=no&watchFile=yes".format("Longitude", "Latitude")
while True:
    vlayer = QgsVectorLayer(uri, "yellow_block", "delimitedtext")
    if vlayer.isValid():
        print("Loading layer...")
        QgsProject.instance().addMapLayer(vlayer)
        break

print("!!!LAYER LOADED!!!")
#Adjusting the extent of canvas
canvas = iface.mapCanvas()
canvas.setExtent(layer.extent())
canvas.refresh()