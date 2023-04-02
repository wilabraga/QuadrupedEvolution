import numpy as np
import quadruped as q
import multiprocessing as mp

freq_bound = 1
force_bound = 5
max_angle_bound = 25
off_bound = 25
phase_bound = 360

num_creatures = 30
p = 0.01
epochs = 1000000
top_k = .6

def create():
    # freq = np.random.random()
    freq = 0.12 # increase for faster gaits
    force = np.random.random() * force_bound
    max_angles = [np.random.random() * max_angle_bound for i in range(8)]
    off = [np.random.random() * off_bound for i in range(8)]
    phases = [np.random.random() * phase_bound for i in range(8)]
    angles = [x for y in [[max_angles[i], off[i], phases[i]] for i in range(8)] for x in y]
    return [freq, force] + angles

def crossover(p1, p2):
    cut1 = np.random.randint(0, len(p1) - 1)
    cut2 = np.random.randint(cut1 + 1, len(p1))
    off1 = p1[0:cut1 + 1] + p2[cut1 + 1:cut2 + 1] + p1[cut2 + 1:]
    off2 = p2[0:cut1 + 1] + p1[cut1 + 1:cut2 + 1] + p2[cut2 + 1:]
    return off1, off2

def mutate(x, p):
    if np.random.random(1) < p:
        i = np.random.randint(0, len(x))
        if i == 0:
            # x[i] = np.random.random()
            x[i] = 0.12
        elif i == 1:
            x[i] = np.random.random() * force_bound
        elif (i-2) % 3 == 0:
            x[i] = np.random.random() * max_angle_bound
        elif (i-2) % 3 == 1:
            x[i] = np.random.random() * off_bound
        else:
            x[i] = np.random.random() * phase_bound
    return x

if __name__ == '__main__':
    creatures = []
    for i in range(epochs):
        for j in range(num_creatures - len(creatures)):
            creatures.append(create())

        #https://stackoverflow.com/questions/9786102/how-do-i-parallelize-a-simple-python-loop
        l = mp.Lock()
        pool = mp.Pool(8, initializer=q.init, initargs=(l,))
        distances = []
        for creature in creatures:
            distances.append(pool.apply_async(q.run, [creature]))
        pool.close()
        pool.join()

        distances = [x.get() for x in distances]

        print('Epoch:', (i+1))
        print('Best:', np.max(distances))
        creatures = [x for _, x in sorted(zip(distances, creatures), reverse=True)]
        print('Params:', creatures[0], '\n')

        new_creatures = [creatures[0], creatures[1]]
        prob = np.asarray(sorted(distances, reverse=True)) - np.min(distances)
        prob /= np.sum(prob)

        for j in range(0, int(top_k * num_creatures) - 1, 2):
            c1, c2 = np.random.choice(np.arange(len(creatures)), 2, replace=False, p=prob)
            off1, off2 = crossover(creatures[c1], creatures[c2])
            new_creatures.append(mutate(off1, p))
            new_creatures.append(mutate(off2, p))

        creatures = new_creatures

