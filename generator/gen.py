#!/usr/bin/python
# -*- coding: utf-8 -*-

from random import random

Ns    = [8,  10, 12, 14, 16, 18, 20, 22]
cases = [20, 20, 20, 20, 10, 10, 10, 10]
for no, n, cc in zip(range(len(Ns)), Ns, cases):
    fp = open("input%.2d.txt" % n, "w")
    fp.write("%d\n" % cc)
    for cc in xrange(cc):
        fp.write("%d\n" % n);
        for i in xrange(n):
            fp.write("%.10lf %.10lf\n" % (random() * 1000, random() * 1000))
    fp.close()
