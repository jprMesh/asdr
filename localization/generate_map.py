
from math import sqrt

# Read in measurements from file
measurements = []
with open('gps_vals_urban.txt', 'r') as gps_vals:
    gps = gps_vals.readlines()
    with open('rss_vals_urban.txt', 'r') as rss_vals:
        rss = rss_vals.readlines()
        rss_scale = len(rss)/len(gps)
        offset_b = 1000
        offset_e = 500
        reduction = 15
        for i in range((len(gps) - offset_b - offset_e) / reduction):
            measurements.append((float(str(gps[offset_b+reduction*i]).split()[0]),
                                 float(str(gps[offset_b+reduction*i]).split()[1]),
                                 float(rss[(offset_b+reduction*i)*rss_scale].strip('\n,'))))

altitude = 100 # in meters
# (Lat, Long, RSSI value)
# measurements = [(42.274744, -71.8084369, -84.3),
#                 (42.275376, -71.8085379, -85.3),
#                 (42.275342, -71.8075235, -85.9)]

# Distance calculation
# Pythagorean theorem to eliminate altitude
rss_dist_meas = [(meas[0], meas[1],
                  (10**((meas[2]/-20))/100))
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