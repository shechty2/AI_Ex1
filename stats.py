'''
This file should be runnable to print map_statistics using 
$ python stats.py
'''

from collections import namedtuple, Counter
from ways import load_map_from_csv


def map_statistics(roads):
    '''return a dictionary containing the desired information
    You can edit this function as you wish'''
    Stat = namedtuple('Stat', ['max', 'min', 'avg'])
    linkCount = len(list(roads.iterlinks()))
    maxBranch = (max(len(junction.links) for junction in roads.junctions()))
    minBranch = (min(len(junction.links) for junction in roads.junctions()))
    avgBranch = linkCount / len(roads)
    maxDist = (max(link.distance for link in roads.iterlinks()))
    minDist = (min(link.distance for link in roads.iterlinks()))
    sumLinks = (sum(link.distance for link in roads.iterlinks()))
    avgDist = sumLinks / linkCount
    return {
        'Number of junctions' : len(roads),
        'Number of links' : linkCount,
        'Outgoing branching factor' : Stat(max=maxBranch, min=minBranch, avg=avgBranch),
        'Link distance' : Stat(max=maxDist, min=minDist, avg=avgDist),
        # value should be a dictionary
        # mapping each road_info.TYPE to the no' of links of this type
        'Link type histogram' : Counter(link.highway_type for link in roads.iterlinks()),  # tip: use collections.Counter
    }
def print_stats():
    for k, v in map_statistics(load_map_from_csv()).items():
        print('{}: {}'.format(k, v))

        
if __name__ == '__main__':
    from sys import argv
    assert len(argv) == 1
    print_stats()
