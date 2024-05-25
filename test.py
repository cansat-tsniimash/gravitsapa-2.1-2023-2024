import numpy

R = 6371
gps_lat = 55.91010002851728
gps_lon = 37.79712189970438
coord_base_lat = 55.913919937024616
coord_base_lon = 37.797132745957676 

dLat = numpy.deg2rad(coord_base_lat - gps_lat)
dLon = numpy.deg2rad(coord_base_lon - gps_lon)

a = numpy.sin(dLat/2) * numpy.sin(dLat/2) + numpy.cos(numpy.deg2rad(gps_lat)) * numpy.cos(numpy.deg2rad(coord_base_lat)) * numpy.sin(dLon/2) * numpy.sin(dLon/2)
c = 2 * numpy.arctan2(numpy.sqrt(a), numpy.sqrt(1-a))
d = R* c
print(d)
# distance = 6400*numpy.arccos(numpy.sin(coord_base_lon)*numpy.sin(gps_lon)*numpy.cos(coord_base_lat-gps_lat)+numpy.cos(coord_base_lon)*numpy.cos(gps_lon))
# print(distance)
