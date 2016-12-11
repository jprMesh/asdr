# Read in measurements from file

# (Lat, Long, RSSI value)
measurements = [(42.274744, -71.8084369, -73.3),
                (42.275376, -71.8085379, -74.3),
                (42.275342, -71.8075235, -74.9)]
# Distance calculation 
rss_dist_meas = [(meas[0], meas[1], 10**((meas[2]/-20))/100)
                 for meas in measurements]

# Populate js list of points to plot
circle = '    {point}: {{center: {{lat: {lat}, lng: {lng}}}, distance: {dist}}}'
circles = [circle.format(point=index,
                         lat=rss_dist_meas[index][0],
                         lng=rss_dist_meas[index][1],
                         dist=rss_dist_meas[index][2],)
           for index in xrange(len(rss_dist_meas))]
mapjs = ',\n'.join(circles)

# Fill in template with data
with open('map_template.html', 'r') as temp:
    html_map = temp.read()
html_map = html_map.format(points=mapjs,
                           center_lat=rss_dist_meas[0][0],
                           center_lng=rss_dist_meas[0][1])
with open('masdr_localization.html', 'w') as f:
    f.write(html_map)