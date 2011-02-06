#!/usr/bin/python
# -*- coding: utf-8 -*-

from random import random

names = ["small", "medium", "large", "xlarge"]
Ns = [10, 15, 20, 25]
for no, name, n in zip(range(len(Ns)), names, Ns):
    fp = open("%d.%s.txt" % (no, name), "w")
    fp.write("20\n")
    for cc in xrange(20):
        fp.write("%d\n" % n);
        for i in xrange(n):
            fp.write("%.10lf %.10lf\n" % (random() * 1000, random() * 1000))
    fp.close()
