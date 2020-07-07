from ways import graph
from random import randrange
import csv


def create_csv(roads):
    with open('problems.csv', 'a',  newline='') as csvfile:
        filewriter = csv.writer(csvfile, delimiter=',')
        j=0
        while j<100:
            routeLenght= randrange(0, 850)
            source= randrange(len(roads))
            while not roads[source].links:
                source = randrange(len(roads))
            n1= source
            for i in range(0, routeLenght):
                if roads[n1].links:
                    n=n1
                    n1= roads[n].links[randrange(0, len(roads[n].links))].target
            if source != n:
                filewriter.writerow([str(source), str(n)])
                j=j+1


r = graph.load_map_from_csv()
create_csv(r)