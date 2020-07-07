import csv

from astar import Node, PriorityQueue, find_astar_rout_search
from ways import draw, graph
import matplotlib.pyplot as plt

roads = graph.load_map_from_csv()  # count=10001)

from ways.draw import set_no_axis, draw_links, plot_path


def read_csv():
    with open('problems.csv', mode='r') as csv_file:
        problems = list()
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            problems.append((row[0], row[1]))
    return problems


j=0
for i in read_csv():
    arr = find_astar_rout_search(int(i[0]), int(i[1]))
    plt.figure(num=1, figsize=(6, 15))
    'set_no_axis()'
    plot_path(roads,arr[0], 'c')
    plt.show()
    plt.clf()