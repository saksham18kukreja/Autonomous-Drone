import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle,Circle

fig, ax = plt.subplots()
ax.set(xlim = (0,10), ylim = (0,10))
path = [(0, 0), (1, 1), (2, 2), (3, 2), (4, 2), (5, 3), (6, 4), (7, 5), (8, 6), (9, 7), (9, 8), (9, 9)]
#plt.figure(figsize=(10,10))
plt.grid(True)
plt.xticks(np.arange(0,11,1))
plt.yticks(np.arange(0,11,1))
rect1 = Rectangle((6.5,2.5),1,1)
rect2 = Rectangle((3.5,3.5),1,1)
rect3 = Rectangle((6.5,6.5),1,1)
rect4 = Rectangle((1.5,7.5),1,1)
#circ_start = Circle((0,0),0.5,color = 'red')
#circ_end = Circle((9,9),0.5,color = 'red')
geom_fin = [rect1,rect2,rect3,rect4]
[ax.add_patch(geo) for geo in geom_fin]
x_point = [x[0] for x in path]
y_point = [x[1] for x in path]
ax.plot(x_point,y_point,'red')
#ax.add_patch(rect1)
#ax.add_patch(rect2)
#ax.add_patch(rect3)
#ax.add_patch(rect4)
#rect = patch.Rectangle((5,3),2,2)
#plt.Rectangle((5,3),2,2)
plt.show()