import octomap

x = open('/Users/anokk/Documents/OCTOMAP_STACK/python-octomap/table.bt','r')

while x.readline()[0] is '#':
    continue

for i in xrange(3):
    print x.readline()
    f, v = x.readline().split(' ', 1)
