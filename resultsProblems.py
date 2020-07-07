

import csv
import os

from idastar import idastar_search
from astar import find_astar_rout_search
from ucs import find_ucs_rout_search
from ucs import CostFunction


def read_csv():
    with open('problems.csv', mode='r') as csv_file:
        problems = list()
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            problems.append((row[0], row[1]))
    return problems


os.makedirs(os.path.dirname("results/UCSRuns.txt"), exist_ok=True)
with open("results/UCSRuns.txt","a") as file:
   for i in read_csv():
       file.write(str(find_ucs_rout_search(int(i[0]), int(i[1]), CostFunction)[0]) + "\n")

os.makedirs(os.path.dirname("results/AStarRuns.txt"), exist_ok=True)
with open("results/AStarRuns.txt","a") as file:
    for i in read_csv():
        arr=find_astar_rout_search(int(i[0]),int(i[1]))
        file.write(str(arr[1])+"\t"+str(arr[2]) + "\n")


os.makedirs(os.path.dirname("results/idastarRuns.txt"), exist_ok=True)
with open("results/idastarRuns.txt", "w") as file:
    j = 1
    for i in read_csv():
        if j < 6:
            arr = idastar_search(int(i[0]), int(i[1]))
            file.write(str(arr[1]) + "\t" + str(arr[2]) + "\n")
            j += 1