#!/usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

plt.scatter(0.5, 0.7, s=200, c='k', marker=(5, 1), lw=2)
plt.axis([0.0,1.0,0.0,1.0])
plt.savefig('star1')

plt.clf()
plt.scatter(0.55, 0.67, s=200, c='k', marker=(5, 1), lw=2)
plt.axis([0.0,1.0,0.0,1.0])
plt.savefig('star2')
