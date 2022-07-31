from re import M
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
# from itertools import chain

# from itertools import chain

# plt.figure(figsize=(8, 8))
# m = Basemap(projection='ortho', resolution=None, lat_0=50, lon_0=-100)
# m.bluemarble(scale=0.5)

# plt.show()


# latitude of lower left hand corner of the desired map domain (degrees).
# llcrnrlat = 59
# # llcrnrlat = 59.3034696183
# # llcrnrlat = -90

# # latitude of upper right hand corner of the desired map domain (degrees).
# urcrnrlat = 60
# # urcrnrlat = 59.3072747372
# # urcrnrlat = 90

# # longitude of lower left hand corner of the desired map domain (degrees).
# llcrnrlon = 18.5
# # llcrnrlon = 18.7066051197
# # llcrnrlon = -180

# # longitude of upper right hand corner of the desired map domain (degrees).
# urcrnrlon = 19
# # urcrnrlon = 18.714404264
# # urcrnrlon = 180

# importing modules
from PIL import Image
import requests
from io import BytesIO

URL = "https://render.openstreetmap.org/cgi-bin/export?bbox=18.704395294189457,59.301644053783804,18.71259212493897,59.30936612300601&scale=4416&format=png"

import urllib, cStringIO

file = cStringIO.StringIO(urllib.urlopen(URL).read())
img = Image.open(file)