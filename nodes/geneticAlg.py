from deap import base
from deap import creator
from deap import tools
from deap import base, algorithms
import random
import numpy as np
import matplotlib.pyplot as plt 
import time

def GeneticAlg():

    def __init__(self):
        # self.RUN = True
        self.WAIT = False

        self.reward = 0

        ONE_MAX_LENGTH = 100 # размер списка, с которым работает ГА
        POPULATION_SIZE = 200 # размер популяции
        P_CROSSOVER = 0.9
        P_MUTATION = 0.1
        MAX_GENERATIONS = 50
        # creator.create("Point", object, MAX_COORD = 10, MIN_COORD = 0, coords=list)

        # pt = creator.Point()
        creator.create("FitnessMax", base.Fitness, weights = (1.0, ))
        creator.create("Individual", list, fitness = creator.FitnessMax)

        def oneMaxFitness(individual):
            self.WAIT = True 
            while(self.WAIT):
                time.sleep(0.1)
            
            return sum(individual), # кортеж

        toolbox = base.Toolbox()
        toolbox.register("randRange", random.randint, 0, 1)
        toolbox.register("individualCreator", tools.initRepeat, creator.Individual, toolbox.randRange, ONE_MAX_LENGTH)
        toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)

        population = toolbox.populationCreator(n=POPULATION_SIZE)

        toolbox.register("select", tools.selTournament, tournsize=3)
        toolbox.register("mate", tools.cxOnePoint)
        toolbox.register("mutate", tools.mutFlipBit, indpb=1.0/ONE_MAX_LENGTH)
        toolbox.register("evaluate", oneMaxFitness)

        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("max", np.max)
        stats.register("avg", np.mean)

        population, logbook = algorithms.eaSimple(population, toolbox,
                                                cxpb=P_CROSSOVER,
                                                mutpb=P_MUTATION,
                                                ngen=MAX_GENERATIONS,
                                                stats=stats,
                                                verbose=True)

        maxFitnessValues, meanFitnessValues = logbook.select("max", "avg")

