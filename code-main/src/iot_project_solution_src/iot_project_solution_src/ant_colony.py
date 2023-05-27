
import math
import random
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from geometry_msgs.msg import Point



# ======================================================
"""
class Point:
    def __init__(self, x=-1, y=-1, z=-1):
        self.x = x if x > 0 else random.uniform(0,50)
        self.y = y if y > 0 else random.uniform(0,50)
        self.z = z if z > 0 else random.uniform(0,50)
    def __repr__(self):
        return str((round(self.x,2),round(self.y,2),round(self.z,2)))
    def __str__(self):
        return '<'+str(round(self.x,2))+','+str(round(self.y,2))+','+str(round(self.z,2))+'>'

"""
# ======================================================



def from_point_to_tuple(p : Point):
    return (p.x,p.y,p.z)
def from_tuple_to_point(p):
    return Point(x=p[0],y=p[1],z=p[2])

def ant_dist(point1, point2,
                aoi2, aoi_threshold2,
                aoi_weight, violation_weight,
                alpha=0.1, beta=1.0, eps=0.0001) -> float:
    """
    Computes the inverse priority of reaching point2 from point1, depending on the current scenario of the simulation.
    
    Takes:
    	point1, point2:   3D tuples
    	aoi_threshold2:   the threshold of the AoI for point2
    	aoi2:             the CURRENT AoI of point2
    	aoi_weight:       the weight of the AoI in the final score
    	violation_weight: the weight of the violation in the final score
    	alpha, beta:      scaling parametrs
    """

    euclidean = math.sqrt( ((point1[0]-point2[0])**2) + ((point1[1]-point2[1])**2) + ((point1[2]-point2[2])**2) )
    if euclidean == 0.0: return eps
    '''
    if (aoi_threshold2 - aoi2) < 0: # constraint violated
        aoi_bonus = (aoi2/aoi_threshold2) * abs(aoi_threshold2 - aoi2)
    elif (aoi_threshold2 - aoi2) == 0: # we don't want to deal with 0
        aoi_bonus = 1.0
    else: # legal
        aoi_bonus = (aoi2/aoi_threshold2) * (1 / (aoi_threshold2 - aoi2))
    
    if euclidean == 0.0: # we are measuring the distance between the same point so it must be 0 regardless of the bonus
        dist = 0.0
    else:
        print("euclidean: " + str(euclidean) + "| aoi_bonus: " + str(aoi_bonus))
        dist = (euclidean*alpha) - (aoi_bonus*beta) if aoi_bonus > 0.0 else (euclidean*alpha)
    '''
    aoi_bonus = aoi2/aoi_threshold2
    
    result = -(aoi_bonus*beta + eps) / (np.exp(euclidean  * alpha))
    return result if (result != 0.0) else eps

def path_length(path,
                  aois, thresholds,
                  aoi_weight, violation_weight,
                  alpha=1.0, beta=1.0, f_dist=ant_dist,
                  loop=False) -> float:
    
    total_len = 0.0
    dist = f_dist

    for i in range(len(path)-1):
        total_len += dist(path[i], path[i+1], aois[path[i+1]], thresholds[path[i+1]], aoi_weight, violation_weight)
    
    if loop:
        total_len += dist(path[-1], path[0], aois[path[0]], thresholds[path[0]], aoi_weight, violation_weight)

    return total_len



# ======================================================



"""
- ants individually keep track of: 
    - `path` list of the points they have previously visited
    - the set of `remaining` points they still need to visit
    - `distance` to their next destination
    - `path_cost` of how many steps they have taken
    - the number of `round_trips` they have made to colony       


- when the ants are travelling, there is a vectorized countdown of `distance`

- when an ant arrives at a node:
    - it randomly selects a new destination on its remaining list
    - its choice is weighted by the amount pheromones left by other ants
    - `pheromone_power` affects the how strongly ants are affected by small differences in pheromones
    - `distance_power`  affects the awareness the ants have of distance, ie a preference to visit closer nodes first
    - `reward_power`    affects how `best_path/current_path` is used when laying new pheromones 
        - `** -3` was counterintuitively discovered as optimal
        - this encourages ants to explore longer paths around the strongest pheromones trail
    - `decay_power`     affects how quickly old pheromone trails decay
    - once an ant has visited all the nodes on its list, it returns home


- once an ant returns home:
    - it updates the `self.pheromones` map for its completed path, telling the others that this way is popular
    - the pheromone reward increases with `self.round_trips` which has the effect of slowly decaying older pheromone trails
    - it updates the `self.ants_used` count    
    - it updates its individual `self.round_trips` count            
    - the ant resets and begins the Travelling Sales Problem again


- if the ant has found a new best-yet path:
    - it informs the queen who keeps a record of the best path
    - the queen doubles the pheromones along this new best path, which doubles probability of ants exploring this path
    - `best_path_smell` controls the multipler the queen uses when a new best path is found 


- Termination: there are several configurable options:
    - `self.stop_factor`   ant logic: if a new best path was found, then redouble efforts in search of a new best path before quitting
        - a factor of 2 will terminate in double the time required to find the last best path
        - a factor of 4 is more reliable in preventing early termination in some cases (but can be very time consuming)
    - `self.time`          run for a fixed number of seconds then stop
    - `self.min_time`      run for a minimum number of seconds before terminating naturally 
    - `self.timeout`       run for a maximum number of seconds but may terminate naturally beforehand
    - `self.round_trips`   wait for a minimum number of round_trips as a precondition to terminating (default 4)
"""



# ======================================================



import time
from itertools import chain
from typing import Any, Callable, List, Tuple, Union
import typing
import numpy as np
import random



class AntColonySolver:
    """
    Initializes the solver, only needs a distance function"""
    def __init__(self,
                 cost_fn:                 Callable[[Any,Any], Union[float,int]],                         
                 
                 time=0,                  # run for a fixed amount of time
                 min_time=0,              # minimum runtime
                 timeout=0,               # maximum time in seconds to run for
                 stop_factor=1,           # how many times to redouble effort after new new best path
                 min_round_trips=10,       # minimum number of round trips before stopping
                 max_round_trips=20,     # maximum number of round trips before stopping                 
                 min_ants=0,              # Total number of ants to use
                 max_ants=0,              # Total number of ants to use
                 
                 ant_count=64,            # 64 is the bottom of the near-optimal range for numpy performance
                 ant_speed=1,             # how many steps do ants travel per epoch

                 distance_power=0,        # power to which distance affects pheromones                 
                 pheromone_power=1.25,    # power to which differences in pheromones are noticed
                 decay_power=0,           # how fast do pheromones decay
                 reward_power=2,          # relative pheromone reward based on best_path_length/path_length 
                 best_path_smell=2,       # queen multiplier for pheromones upon finding a new best path                  
                 start_smell=0,           # amount of starting pheromones [0 defaults to `10**self.distance_power`]

                 verbose=False,
                 alpha=1.0,
                 beta=1.0

    ):
        assert callable(cost_fn)        
        self.cost_fn         = cost_fn
        self.time            = int(time)
        self.min_time        = int(min_time)
        self.timeout         = int(timeout)
        self.stop_factor     = float(stop_factor)
        self.min_round_trips = int(min_round_trips)
        self.max_round_trips = int(max_round_trips)
        self.min_ants        = int(min_ants)
        self.max_ants        = int(max_ants)
    
        self.ant_count       = int(ant_count)
        self.ant_speed       = int(ant_speed)
        
        self.distance_power  = float(distance_power)     
        self.pheromone_power = float(pheromone_power)
        self.decay_power     = float(decay_power)
        self.reward_power    = float(reward_power)
        self.best_path_smell = float(best_path_smell)
        self.start_smell     = float(start_smell or 10**self.distance_power)
        
        self.verbose         = int(verbose)
        self._initalized     = False

        self.alpha = alpha
        self.beta = beta
        
        if self.min_round_trips and self.max_round_trips:
            self.min_round_trips = min(self.min_round_trips, self.max_round_trips)
        if self.min_ants and self.max_ants:
            self.min_ants = min(self.min_ants, self.max_ants)


    def solve_initialize(
            self,
            problem_path: List[Any], 
            aois, thresholds,
            aoi_weight : float, violation_weight : float,
            alpha=1.0, beta=1.0
    ) -> None:

        ### Cache of distances between nodes
        self.distances = {
            source: {
                dest: self.cost_fn(source, dest, aois[dest], thresholds[dest], aoi_weight, violation_weight, alpha=alpha, beta=beta)
                for dest in problem_path
            }
            for source in problem_path
        }

        ### Cache of distance costs between nodes - division in a tight loop is expensive
        self.distance_cost = {
            source: {
                dest: 1 / (1 + self.distances[source][dest]) ** self.distance_power
                for dest in problem_path
            }
            for source in problem_path
        }

        ### This stores the pheromone trail that slowly builds up
        self.pheromones = {
            source: {
                # Encourage the ants to start exploring in all directions and furthest nodes
                dest: self.start_smell
                for dest in problem_path
            }
            for source in problem_path
        }
        
        ### Sanitise input parameters
        if self.ant_count <= 0:
            self.ant_count = len(problem_path)
        if self.ant_speed <= 0:
            self.ant_speed = np.median(list(chain(*[ d.values() for d in self.distances.values() ]))) // 5
        self.ant_speed = int(max(1,self.ant_speed))
        
        ### Heuristic Exports
        self.ants_used   = 0
        self.epochs_used = 0
        self.round_trips = 0
        self._initalized = True        


    def solve(self,
              problem_path: List[Any],
              aois, thresholds,
              aoi_weight : float, violation_weight : float,
              restart=False,
              alpha=1.0, beta=1.0
    ) -> List[Tuple[int,int]]:
    
        if restart or not self._initalized:
            self.solve_initialize(problem_path, aois, thresholds, aoi_weight, violation_weight, alpha=alpha, beta=beta)

        ### Here come the ants!
        ants = {
            "distance":    np.zeros((self.ant_count,)).astype('int32'),
            "path":        [ [ problem_path[0] ]   for n in range(self.ant_count) ],
            "remaining":   [ set(problem_path[1:]) for n in range(self.ant_count) ],
            "path_cost":   np.zeros((self.ant_count,)).astype('int32'),
            "round_trips": np.zeros((self.ant_count,)).astype('int32'),
        }

        best_path       = None
        best_path_cost  = np.inf
        best_epochs     = []
        epoch           = 0
        time_start      = time.perf_counter()
        while True:
            epoch += 1

            ### Vectorized walking of ants
            # Small optimization here, testing against `> self.ant_speed` rather than `> 0` 
            #       avoids computing ants_arriving in the main part of this tight loop
            ants_travelling = (ants['distance'] > self.ant_speed)
            ants['distance'][ ants_travelling ] -= self.ant_speed
            if all(ants_travelling):
                continue  # skip termination checks until the next ant arrives
            
            ### Vectorized checking of ants arriving
            ants_arriving       = np.invert(ants_travelling)
            ants_arriving_index = np.where(ants_arriving)[0]
            for i in ants_arriving_index:

                ### ant has arrived at next_node
                this_node = ants['path'][i][-1]
                next_node = self.next_node(ants, i)
                ants['distance'][i]  = self.distances[ this_node ][ next_node ]
                ants['remaining'][i] = ants['remaining'][i] - {this_node}
                ants['path_cost'][i] = ants['path_cost'][i] + ants['distance'][i]
                ants['path'][i].append( next_node )

                ### ant has returned home to the colony
                if not ants['remaining'][i] and ants['path'][i][0] == ants['path'][i][-1]:
                    self.ants_used  += 1
                    self.round_trips = max(self.round_trips, ants["round_trips"][i] + 1)

                    ### We have found a new best path - inform the Queen
                    was_best_path = False
                    if ants['path_cost'][i] < best_path_cost:
                        was_best_path  = True
                        best_path_cost = ants['path_cost'][i]
                        best_path      = ants['path'][i]
                        best_epochs   += [ epoch ]
                        if self.verbose:
                            print({
                                "path_cost":   int(ants['path_cost'][i]),
                                "ants_used":   self.ants_used,
                                "epoch":       epoch,
                                "round_trips": ants['round_trips'][i] + 1,
                                "clock":       int(time.perf_counter() - time_start),
                            })

                    ### leave pheromone trail
                    # doing this only after ants arrive home improves initial exploration
                    #  * self.round_trips has the effect of decaying old pheromone trails
                    # ** self.reward_power = -3 has the effect of encouraging ants to explore longer routes
                    #                           in combination with doubling pheromone for best_path
                    reward = 1
                    if self.reward_power: reward *= ((best_path_cost / ants['path_cost'][i]) ** self.reward_power)
                    if self.decay_power:  reward *= (self.round_trips ** self.decay_power)
                    for path_index in range( len(ants['path'][i]) - 1 ):
                        this_node = ants['path'][i][path_index]
                        next_node = ants['path'][i][path_index+1]
                        self.pheromones[this_node][next_node] += reward
                        self.pheromones[next_node][this_node] += reward
                        if was_best_path:
                            # Queen orders to double the number of ants following this new best path                            
                            self.pheromones[this_node][next_node] *= self.best_path_smell
                            self.pheromones[next_node][this_node] *= self.best_path_smell


                    ### reset ant
                    ants["distance"][i]     = 0
                    ants["path"][i]         = [ problem_path[0] ]
                    ants["remaining"][i]    = set(problem_path[1:])
                    ants["path_cost"][i]    = 0
                    ants["round_trips"][i] += 1


            ### Do we terminate?
            
            # Always wait for at least 1 solutions (note: 2+ solutions are not guaranteed)
            if not len(best_epochs): continue 
            
            # Timer takes priority over other constraints
            if self.time or self.min_time or self.timeout:
                clock = time.perf_counter() - time_start
                if self.time:
                    if clock > self.time: break
                    else:                 continue
                if self.min_time and clock < self.min_time: continue
                if self.timeout  and clock > self.timeout:  break
            
            # First epoch only has start smell - question: how many epochs are required for a reasonable result?
            if self.min_round_trips and self.round_trips <  self.min_round_trips: continue        
            if self.max_round_trips and self.round_trips >= self.max_round_trips: break

            # This factor is most closely tied to computational power                
            if self.min_ants and self.ants_used <  self.min_ants: continue        
            if self.max_ants and self.ants_used >= self.max_ants: break            
            
            # Lets keep redoubling our efforts until we can't find anything more
            if self.stop_factor and epoch > (best_epochs[-1] * self.stop_factor): break
                                
            # Nothing else is stopping us: Queen orders the ants to continue!      
            if True: continue
            
            
            
        ### We have (hopefully) found a near-optimal path, report back to the Queen
        self.epochs_used = epoch
        self.round_trips = np.max(ants["round_trips"])
        return best_path


    def next_node(self, ants, index):
        this_node   = ants['path'][index][-1]

        weights     = []
        weights_sum = 0
        if not ants['remaining'][index]: return ants['path'][index][0]  # return home
        for next_node in ants['remaining'][index]:
            if next_node == this_node: continue
            reward = (
                    self.pheromones[this_node][next_node] ** self.pheromone_power
                    * self.distance_cost[this_node][next_node]  # Prefer shorter paths
            )
            weights.append( (reward, next_node) )
            weights_sum   += reward

        # Pick a random path in proportion to the weight of the pheromone
        rand = random.random() * weights_sum
        for (weight, next_node) in weights:
            if rand > weight: rand -= weight
            else:             break
        return next_node
            
        
def AntColonyRunner(points, aois, thresholds, aoi_weight, violation_weight, dist_fn=ant_dist, ant_count=64, alpha=1.0, beta=1.0, verbose=False, plot=False, label={}, **kwargs):
    solver     = AntColonySolver(cost_fn=dist_fn, verbose=verbose, ant_count=ant_count, alpha=alpha, beta=beta, **kwargs)
    start_time = time.perf_counter()
    result     = solver.solve(points, aois, thresholds, aoi_weight, violation_weight)
    stop_time  = time.perf_counter()
    if label: kwargs = { **label, **kwargs }
        
    for key in ['verbose', 'plot', 'animate', 'label', 'min_time', 'max_time']:
        if key in kwargs: del kwargs[key]
    print("N={:<3d} | {:5.0f} -> {:4.0f} | {:4.0f}s | ants: {:5d} | trips: {:4d} | "
          .format(len(points), path_length(points, aois, thresholds, aoi_weight, violation_weight), path_length(result, aois, thresholds, aoi_weight, violation_weight), (stop_time - start_time), solver.ants_used, solver.round_trips)
          + " ".join([ f"{k}={v}" for k,v in kwargs.items() ])
    )
    return result


def preprocess_set(Points : List[Any],
                   aois : List[Any], thresholds : List[Any],
                   aoi_weight : float, violation_weight : float,
                   drone_pos : Point,
                   dist_fn=ant_dist):

    ### Convert points to tuples
    new_problem = []
    for point in Points:
        new_problem.append(from_point_to_tuple(point))

    ### pfind the closest point to the current position of the drone
    p0 = from_point_to_tuple(drone_pos)
    closest_i = 0
    min_dist = float("inf")
    for i in range(len(new_problem)):
        dist = dist_fn(p0, new_problem[i], aois[i], thresholds[i], aoi_weight, violation_weight)
        if dist < min_dist:
            closest_i = i
            min_dist = dist
    
    ### put the closest point as the start of the sequence
    tmp = new_problem[0]
    new_problem[0] = new_problem[closest_i]
    new_problem[closest_i] = tmp

    ### update the aois
    tmp = aois[0]
    aois[0] = aois[closest_i]
    aois[closest_i] = tmp

    ### update the thresholds
    tmp = thresholds[0]
    thresholds[0] = thresholds[closest_i]
    thresholds[closest_i] = tmp

    ### convert aois to dictionary form
    dict_aois = { new_problem[i] : aois[i] for i in range(len(Points)) }

    ### convert thresholds to dictionary form
    dict_thresholds = { new_problem[i] : thresholds[i] for i in range(len(Points)) }

    ### return preprocessed problem
    return new_problem, dict_aois, dict_thresholds

def postprocess_path(Path : List[Any], path_len):
    new_path = []
    for point in Path:
        new_path.append(from_tuple_to_point(point))
    return new_path[:path_len]

def find_patrol_route(Env : List[Any], aois : List[Any], thresholds : List[Any],
               aoi_weight : float, violation_weight : float, drone_pos : Point,
               distance_fn=ant_dist,
               complete_path=True,
               path_len=100,
               loop=False,
               alpha=1.0,
               beta=1.0,
               ant_count=64):
    """
    This function initializes the problem and feeds it to an ant colony that will try to find the best path.
    If complete_path=False the function will return ONLY the first path_len steps (i.e. a partial path).

    Arguments:
        Env              : deterministically ordered list of Points
        aois             : deterministically ordered list of Age of Information for the Points in Env
        thresholds       : deterministically ordered list of AoI Thresholds for the Poinnts in Env
        aoi_weight       : weight for the AoI in the simulation score
        violation_weight : weight for the violation in the simulation score
        drone_pos        : position of the drone at the start of the path,
                           this will be used to map the beginning of the path
                           to the point closest to the drone
        distance_fn      : function used to evaluate distances between points.
                           the points must be either 3D arrays or tuples, not ROS Point types.
                           The goal will be to MINIMIZE this function.
        complete_path    : boolean flag to signal the algorithm if we want only a partial path or not
        path_len         : if we want only a partial path, this will be the length of said path.
        loop             : if this is true then the function will extend the path to include a "reverse"
                           path back to the origin following the same steps backwards.
    """
    ### variables
    n_points = len(Env)

    ### pre-processing
    Points, dict_aois, dict_thresholds = preprocess_set(Env, aois, thresholds, aoi_weight, violation_weight, drone_pos)

    ### solve problem
    result = AntColonyRunner(Points, dict_aois, dict_thresholds, aoi_weight, violation_weight, dist_fn=distance_fn, alpha=alpha, beta=beta, ant_count=ant_count)

    ### post-processing
    path = postprocess_path(result, n_points) if complete_path else postprocess_path(result, min(n_points,path_len))

    if loop:
        i = len(path)-2
        while (i >= 0):
            path.append(path[i])
            i -= 1

    return path



# ====================================================== TEST

"""
n = 50
random.seed(0)
Env = [Point() for i in range(n)]
aois = [random.uniform(0,50) for i in range(n)]
thresholds = [random.uniform(0,50) for i in range(n)]
aoi_w = random.uniform(0,1)
v_w = random.uniform(0,1)
drone_pos = Point(x=0.0, y=0.0, z=0.0)
ALPHA = 1.0
BETA = 0.1
ANTS = 32

path = find_patrol_route(Env,aois,thresholds,aoi_w, v_w,drone_pos, loop=True, alpha=ALPHA, beta=BETA,ant_count=ANTS)
print(path)
"""
