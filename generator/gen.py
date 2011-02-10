#!/usr/bin/python
# -*- coding: utf-8 -*-

from random import random

names = ["small", "medium", "large"]
Ns = [10, 16, 24]
cc = 20
for no, n, name in zip(range(len(Ns)), Ns, names):
    fp = open("input%.2d%s.txt" % (n, name), "w")
    fp.write("%d\n" % cc)
    for cc in xrange(cc):
        fp.write("%d\n" % n);
        for i in xrange(n):
            fp.write("%.10lf %.10lf\n" % (random() * 1000, random() * 1000))
    fp.close()
