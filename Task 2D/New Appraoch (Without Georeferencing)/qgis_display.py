import os
from qgis.core import QgsVectorLayer
from tkinter import * 
import csv

## Directory for address of files.... Update it before running script... !!!!!!!!!!!!!!!!!!!!!
address = "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/Working/"
# Update it!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# Address of the task2d.tif file
file = address + "task2d.tif"

# Address of the plot.csv file
csv_path = address + "plot.csv"

# Symbol Properties
symbol_prop = {'angle': '0', 'color': '219,30,42,255', 'horizontal_anchor_point': '1', 'joinstyle': 'bevel', 'name': 'circle', 'offset': '0,0', 'offset_map_unit_scale': '3x:0,0,0,0,0,0', 'offset_unit': 'MM', 'outline_color': '128,17,25,255', 'outline_style': 'solid', 'outline_width': '0.4', 'outline_width_map_unit_scale': '3x:0,0,0,0,0,0', 'outline_width_unit': 'MM', 'scale_method': 'diameter', 'size': '4', 'size_map_unit_scale': '3x:0,0,0,0,0,0', 'size_unit': 'MM', 'vertical_anchor_point': '1'}

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
vlayer = QgsRasterLayer(file, baseName)  

# Creating a .csv file with the sample data 
data = [['id', 'Longitude', 'Latitude'],["obj",77.28,6.40]]

# Writing sample data to csv file
with open(csv_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    print("Writing data to csv file...")
    writer.writerows(data)

#Attaching the .csv file, that gets updated with location when a new block is found...
uri = r"file://{}?type=csv&detectTypes=yes&xField={}&yField={}&crs=EPSG:4326&spatialIndex=no&subsetIndex=no&watchFile=yes".format(csv_path,"Longitude", "Latitude")

#Loading the .csv file as a layer
while True:
    layer = QgsVectorLayer(uri, "yellow_block", "delimitedtext")
    # create a new symbol
    symbol = QgsMarkerSymbol.createSimple(symbol_prop)
    
    # apply symbol to layer renderer
    layer.renderer().setSymbol(symbol)

    # repaint the layer
    layer.triggerRepaint()
    
    # Load layer
    if layer.isValid():
        print("Loading layer...")
        QgsProject.instance().addMapLayer(layer)
        break
    

print("!!!LAYER LOADED!!!")

#Adjusting the extent of canvas
canvas = iface.mapCanvas()
canvas.setExtent(vlayer.extent())
canvas.refresh()
