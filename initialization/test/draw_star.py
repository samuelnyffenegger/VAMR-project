#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

n = 1
x = 0.3 # np.random.rand(n)
y = 0.2 # np.random.rand(n)
z = np.sqrt(x**2 + y**2)

plt.scatter(0.5, 0.7, s=200, c='k', marker=(5, 1), lw=2)
plt.axis([0.0,1.0,0.0,1.0])
plt.savefig('star1')

plt.clf()
plt.scatter(0.55, 0.6, s=200, c='k', marker=(5, 1), lw=2)
plt.axis([0.0,1.0,0.0,1.0])
plt.savefig('star2')
