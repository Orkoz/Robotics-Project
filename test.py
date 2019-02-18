import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import Queue
from time import time
#
#
# class Cord:
#     def __init__(self, priority, loc_x, loc_y):
#         self.priority = priority
#         self.x = loc_x
#         self.y = loc_y
#         print 'cor:', str(self.x) + "," + str(self.y) + "," + str(self.priority)
#         return
#
#     def __cmp__(self, other):
#         return cmp(self.priority, other.priority)
#
#     def __str__(self):
#         return str(self.x) + "," + str(self.y) + "," + str(self.priority)
#
# q = Queue.PriorityQueue()
#
# q.put(Cord(6, 7, 7))
# q.put(Cord(7, 8, 7))
# q.put(Cord(6, 7, 8))
#
# while not q.empty():
#     next_cor = q.get()
#     print 'order of corr:', next_cor


#
# fig, ax = plt.subplots()
#
# min_val, max_val = 0, 15
#
# intersection_matrix = np.random.randint(0, 10, size=(max_val, max_val))
#
# ax.matshow(intersection_matrix, cmap=plt.cm.Blues)
#
# for i in xrange(15):
#     for j in xrange(15):
#         c = intersection_matrix[j,i]
#         ax.text(i, j, str(c), va='center', ha='center')
#
# plt.show()

a = "robot " + str(time()) + " .csv"
print(a)
# temp_x = 1
# temp_y = 2
# x = []
# y = []
# plt.ion()
#
# while True:
#     x.append(temp_x)
#     y.append(temp_x+1)
#     plt.plot(x, y)
#     plt.show(block=False)
#     sleep(2)
#     plt.close()
#     temp_x += 1
#     a = x[-1]
#     print(a)
#
