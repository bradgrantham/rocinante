#!/usr/bin/env python3
import sys

# expects geometric shape
# one triangle per line
# three vertices per triangle separated by whitespace
# vertex is x y z nx ny nz separated by whitespace

vertToIndex = {}
verts = []
triangles = []

for l in sys.stdin:
    parts = l.strip().split()
    v0 = " ".join(parts[0:6])
    v1 = " ".join(parts[6:12])
    v2 = " ".join(parts[12:18])

    if v0 not in vertToIndex:
        n = len(verts)
        vertToIndex[v0] = n
        verts.append(v0)
    i0 = vertToIndex[v0]

    if v1 not in vertToIndex:
        n = len(verts)
        vertToIndex[v1] = n
        verts.append(v1)
    i1 = vertToIndex[v1]

    if v2 not in vertToIndex:
        n = len(verts)
        vertToIndex[v2] = n
        verts.append(v2)
    i2 = vertToIndex[v2]

    triangles.append((i0, i1, i2))

print("%d" % (len(verts)))
for v in verts:
    print("%s" % (v))

print("%d" % (len(triangles)))
which = 0
for t in triangles:
    if which % 2 == 0:
        print("%s %s %s" % (t[0], t[1], t[2]))
    else:
        print("%s %s %s" % (t[0], t[2], t[1]))
    which = which + 1
