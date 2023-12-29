import esda
import os
import osmnx as ox
import networkx as nx
import numpy as np
import pandas as pd
from splot.esda import moran_scatterplot, lisa_cluster, plot_moran
import geopandas as gpd
import spaghetti as spt
import matplotlib.pyplot as plt 
from pyproj import CRS, Transformer
from shapely.geometry import Polygon, Point, LineString
import matplotlib_scalebar
from matplotlib_scalebar.scalebar import ScaleBar

# Task A
# 1.1 data processing
# choose research graph
research_graph = ox.graph_from_point((53.7944140, -1.5486210), dist=600, network_type='drive', simplify = True)
# plot the graph
ox.plot_graph (research_graph)
# project the network from WGS84 to Universal Transverse Mercator Grid System
research_graph_projected = ox.project_graph(research_graph)
# transform the projected graph into geodataframe
research_nodes, research_edges = ox.graph_to_gdfs(research_graph_projected, edges =True, nodes = True)
research_area = research_edges.unary_union.convex_hull.area # Get sized area of the research graph
print('The area of research graph is {} square meters.'.format(research_area))

# read data sets of accidents from 2015 to 2019
DATA_PATH = "data_NDA/"
FILE_NAMES = [ "2015.csv", "2016.csv", "2017.csv", "2018.csv", "2019.csv"]

# use the Grid reference: Northing and Easting in the data
USE_COLS = [1,2]
# read in the accident data from 2009 to 2019 as a Pandas DataFrame
all_accidents = pd.DataFrame(columns=['Easting', 'Northing'])
for FILE_NAME in FILE_NAMES:
    accidents = pd.read_csv (DATA_PATH+FILE_NAME, delimiter = ",",  header=0, names=['Easting', 'Northing'], usecols = USE_COLS)
    all_accidents=pd.concat([all_accidents, accidents])
# drop some accidents which do not record a location that have 'NaN' for the longitude or latitude
located_accidents= all_accidents.dropna (subset = ['Easting', 'Northing'])
located_accidents.head()

# transform from the British National Grid to the WGS84
os.environ['PROJ_NETWORK'] = 'OFF'
def transformBNG2WGS84(East, North):
    crs_british = CRS.from_epsg(27700) # British national grid
    crs_wgs84 = CRS.from_epsg(4326) # WGS84
    transformer = Transformer.from_crs(crs_british, crs_wgs84)
    xx, yy = transformer.transform(East, North)
    return xx, yy
located_accidents.Easting, located_accidents.Northing = transformBNG2WGS84(located_accidents.Easting, located_accidents.Northing)
located_accidents=located_accidents.rename(columns = {"Easting": "Latitude", "Northing":"Longitude"})

# transform to a GeoDatFrame where the geometry is given by a Point constructed from the longitude and latitude
accidents_points = gpd.GeoDataFrame (geometry = [Point(xy) for xy in zip (located_accidents['Longitude'], located_accidents['Latitude'])])

# select and count the accidents in research area
nodes_wgs, edges_wgs = ox.graph_to_gdfs(research_graph, edges =True, nodes = True)
area_polygon = edges_wgs.unary_union.convex_hull # creat a polygon of this area
research_accidents = accidents_points[accidents_points.geometry.within (area_polygon)]
print("There are {} accidents during five years.".format(research_accidents.shape[0]))

# 1.2 The characteristic properties of the network
# check the planarity of the research graph
print("The graph is planar:",nx.check_planarity (research_graph)[0])

# get the basic statistical characteristics
basic_stats = ox.basic_stats(research_graph_projected, area = research_area)
basic_stats

# get the extended statistics
extended_stats = ox.extended_stats(research_graph_projected, ecc=False, bc=True, cc=True)
# turn them into pandas series
for key, value in extended_stats.items():
    basic_stats[key] =value
stats_def = pd.Series(basic_stats)
stats_def

# find the most important node
max_node = stats_def.pagerank_max_node
# get its geometry
key_node = node["geometry"][max_node]
key_node = gpd.GeoDataFrame(geometry = [key_node])

# show the network
base_network = edge.plot (color = "k", zorder = 0, figsize = (12, 12), linewidth=3, alpha=0.25)
nodes_wgs.plot(ax=base_network, markersize=20, color="black", zorder=1)
# Plot the key node on the road network
key_node.plot(marker = "o", ax = base_network, markersize=150, alpha=1, color="r", zorder=2)

# Task B
# 2.1 The distribution of road accidents in the network
# get the locations of the points
x_values = nx.get_node_attributes (research_graph, 'x')
y_values = nx.get_node_attributes (research_graph, 'y')

# work with the edges and add the missing geometries (lines denoting the roads between points)
graph_with_geometries = list (research_graph.edges (data=True))

# iterate through the edges and, where missing, add a geometry attribute with the line between start and end nodes
for e in graph_with_geometries:
    if not 'geometry' in e[2]:
        e[2]['geometry'] = LineString ([Point (x_values[e[0]], y_values[e[0]]), Point (x_values[e[1]], y_values[e[1]])])

# drop the start and end nodes and construct a new Spaghetti network based on the geometries of the roads
road_lines = [x[2] for x in graph_with_geometries]
# construct a GeoDataFrame
roads_geodataframe = gpd.GeoDataFrame(pd.DataFrame (road_lines))
accidents_points_graph = spt.Network(in_data = roads_geodataframe, extractgraph=True)

# snap the crime points to the network
accidents_points_graph.snapobservations (research_accidents, 'accidents')

# show the network
base_network = edges_df.plot (color = "k", zorder = 0, figsize = (12, 12), linewidth=3, alpha=0.25)
# get a GeoDataFrame of the snapped crime locations to plot on the network image
snapped_accidents = spt.element_as_gdf (accidents_points_graph, pp_name = 'accidents', snapped = True)
# plot these on the road network
snapped_accidents.plot (color = "r", marker = "x", markersize = 50, zorder = 1, ax = base_network)
plt.show ()

# 2.2 Spatial autocorrelation of road accidents
# a. Global Moran I
# compute the counts
pointpat = accidents_points_graph.pointpatterns['accidents']
counts = accidents_points_graph.count_per_link (pointpat.obs_to_arc, graph = False)

# compute the weights and values
weights = accidents_points_graph.w_network
values = [counts[i] if i in counts else 0.0 for i in weights.neighbors] 

# compute moran index
moran_accidents = esda.moran.Moran(values, weights)
print("The value of moran I is {}, with the p-value of {} and the z-score of {}.".format(moran.I, moran.p_sim, moran.z_sim))

# plot the moran scatterplot and reference distribution
figsize = (12,6)
plot_moran(moran_accidents, zstandard=True, figsize=figsize)


# 2.3 The specific location of the accident happening on a street segment 
# compute the fraction of the road length away from the nearest intersection accidents occur
fraction_list = []
# distances betweet accident points and vertices
dist_accidents = accidents_points_graph.pointpatterns["accidents"].dist_to_vertex
for key_1,values_1 in  dist_accidents.items():
    dist = 0
    dist_right = 0
    for key_2, values_2 in values_1.items():
        dist = values_2 + dist
        dist_right = values_2 # distances between accident points and right intersections
    dist_left = dist - dist_right # distances between accident points and left intersections
    min_dist = min([dist_left, dist_right]) # distances between accident points and nearest intersections
    fraction = min_dist / dist 
    fraction_list.append(fraction)
# Compute the number of accidents happening nearer to the intersection
accs_near_intersection = [x for x in fraction_list if x < 0.25]
# Compute the number of accidents happening on the open street
accs_along_road = [x for x in fraction_list if x >= 0.25]
print("The number of accidents happening nearer to the intersection: {}, while the number of accidents happening nearer to partway along the road: {}".format(len(accs_near_intersection), len(accs_along_road)))

# plot a pie chart for it
plt.figure(figsize=(12,9))
data = [len(accs_near_intersection), len(accs_along_road)]
label_name = ['Accidents near the intersection','Accidents near the partway']
plt.pie(data,labels=label_name, labeldistance = 1.1, autopct='%1.2f%%', startangle=90)
plt.axis('equal')
plt.legend()
