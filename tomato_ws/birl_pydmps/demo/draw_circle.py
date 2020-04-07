from matplotlib import pyplot
from shapely.geometry.polygon import LinearRing, Polygon

poly = Polygon([(0, 0), (0, 2), (1, 1),
    (2, 2), (2, 0), (1, 0.8), (0, 0)])
x,y = poly.exterior.xy

ax = fig.add_subplot(111)
ax.plot(x, y, color='#6699cc', alpha=0.7,
    linewidth=3, solid_capstyle='round', zorder=2)
ax.set_title('Polygon')

