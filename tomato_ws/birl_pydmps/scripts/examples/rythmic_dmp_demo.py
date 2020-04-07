#!/usr/bin/env python
import matplotlib.pyplot as plt
from birl_pydmps import DMPs_rhythmic
import numpy as np

# test normal run
dmp = DMPs_rhythmic(n_dmps=1, n_bfs=10, w=np.zeros((1, 10)))
y_track, dy_track, ddy_track = dmp.rollout()

plt.figure(1, figsize=(6, 3))
plt.plot(np.ones(len(y_track))*dmp.goal, 'r--', lw=2)
plt.plot(y_track, lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['goal', 'system state'], loc='lower right')
plt.tight_layout()

# test imitation of path run
plt.figure(2, figsize=(6, 4))
n_bfs = [10, 30, 50, 100, 10000]

path1 = np.sin(np.arange(0, 2*np.pi, .01)*5)
# a strange path to target
path2 = np.zeros(path1.shape)
path2[int(len(path2) / 2.):] = .5

for ii, bfs in enumerate(n_bfs):
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=bfs)

    dmp.imitate_path(y_des=np.array([path1, path2]))
    y_track, dy_track, ddy_track = dmp.rollout()

    plt.figure(2)
    plt.subplot(211)
    plt.plot(y_track[:, 0], lw=2)
    plt.subplot(212)
    plt.plot(y_track[:, 1], lw=2)

plt.subplot(211)
a = plt.plot(path1, 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend([a[0]], ['desired path'], loc='lower right')
plt.subplot(212)
b = plt.plot(path2, 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['%i BFs' % i for i in n_bfs], loc='lower right')

plt.tight_layout()
plt.show()
