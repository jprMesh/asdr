# Read in measurements from file
# (Lat, Long, Distance in meters)
measurements = [(42.274744, -71.8084369, 54),
                (42.275376, -71.8085379, 43),
                (42.275342, -71.8075235, 52)]

circle = '    {point}: {{center: {{lat: {lat}, lng: {lng}}}, distance: {dist}}}'
circles = [circle.format(point=index,
                         lat=measurements[index][0],
                         lng=measurements[index][1],
                         dist=measurements[index][2],)
           for index in xrange(len(measurements))]
mapjs = ',\n'.join(circles)

with open('map_template.html', 'r') as temp:
    html_map = temp.read()
html_map = html_map.format(points=mapjs,
                           center_lat=measurements[0][0],
                           center_lng=measurements[0][1])
with open('masdr_localization.html', 'w') as f:
    f.write(html_map)