
import csv
from astar import find_astar_rout_search
from ucs import find_ucs_rout_search, CostFunction
from ways.tools import timed
from ways import graph


def read_csv():
    with open('problems.csv', mode='r') as csv_file:
        problems = list()
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            problems.append((row[0], row[1]))
    return problems

@timed
def timeUcs():
   for i in read_csv():
       find_ucs_rout_search(int(i[0]), int(i[1]), CostFunction)


@timed
def timeAstar():
    for i in read_csv():
        find_astar_rout_search(int(i[0]), int(i[1]))


@timed
def upload_time():
        graph.load_map_from_csv()

timeUcs()
timeAstar()
upload_time()