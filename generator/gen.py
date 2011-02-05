#!/usr/bin/python
# -*- coding: utf-8 -*-

from random import random

MAX_N = 50
for n in xrange(1,MAX_N):
    fp = open("input%.2d.txt" % n, "w")
    fp.write("%d\n" % n);
    for i in xrange(n):
        fp.write("%.10lf %.10lf\n" % (random() * 1000, random() * 1000))
    fp.close()
