import numpy as np
import math
import random
import matplotlib.pyplot as plt
import matplotlib.image as mpimg



# ======================================================



class Point:
    def __init__(self, x=-1, y=-1, z=-1):
        self.x = x if x > 0 else random.uniform(0,9)
        self.y = y if y > 0 else random.uniform(0,9)
        self.z = z if z > 0 else random.uniform(0,9)
    def __repr__(self):
        return str((round(self.x,2),round(self.y,2),round(self.z,2)))
    def __str__(self):
        return '<'+str(round(self.x,2))+','+str(round(self.y,2))+','+str(round(self.z,2))+'>'
    

# ======================================================



def calculate_target_priority(point1 : Point, point2 : Point,
                              aoi2 : float, aoi_threshold2 : float,
                              aoi_weight : float, violation_weight : float,
                              alpha=1.0, beta=1.0) -> float:
    """
    Computes the inverse priority of reaching point2 from point1, depending on the current scenario of the simulation.
    
    Takes:
    	point1, point2: ROS points
    	aoi_threshold2: the threshold of the AoI for point2
    	aoi2:           the CURRENT AoI of point2
    	aoi_weight:     the weight of the AoI in the final score
    	violation_weight: the weight of the violation in the final score
    	alpha, beta: scaling parametrs
    """
    p1 = np.array((point1.x, point1.y, point1.z))
    p2 = np.array((point2.x, point2.y, point2.z))
    
    euclidean = np.linalg.norm(p1 - p2)
    ### debug
    #print("[MESSAGE] Euclidean norm: %s" % euclidean)
    if violation_weight > aoi_weight:
        if (aoi_threshold2 - aoi2) < 0: # constraint violated
            aoi_bonus = (aoi2/aoi_threshold2) * abs(aoi_threshold2 - aoi2)
        elif (aoi_threshold2 - aoi2) == 0: # we don't want to deal with 0
            aoi_bonus = 1.0
        else: # legal
            aoi_bonus = (aoi2/aoi_threshold2) * (1 / (aoi_threshold2 - aoi2))
    else:
        aoi_bonus = aoi2
    
    if euclidean == 0.0:
        priority = 0.0
    else:
        priority = -(aoi_bonus*beta)/(euclidean*alpha) if aoi_bonus > 0.0 else (euclidean*alpha)
    ### debug
    #print("[MESSAGE] Priority:", -priority)
    return priority

def from_point_to_tuple(p : Point):
    return (p.x,p.y,p.z)
def from_tuple_to_point(p):
    return Point(p[0],p[1],p[2])

def tuple_dist(point1, point2,
                aoi2=0.0, aoi_threshold2=0.0,
                aoi_weight=0.0, violation_weight=0.0,
                alpha=1.0, beta=1.0, dist_fn=calculate_target_priority) -> float:
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
    p1 = from_tuple_to_point(point1)
    p2 = from_tuple_to_point(point2)
    return calculate_target_priority(p1,p2, aoi2, aoi_threshold2, aoi_weight, violation_weight, alpha=alpha, beta=beta)

def path_lenght(path,
                  aoi2=0.0, aoi_threshold2=0.0,
                  aoi_weight=0.0, violation_weight=0.0,
                  alpha=1.0, beta=1.0, f_dist=tuple_dist,
                  loop=False) -> float:
    
    total_len = 0.0
    dist = f_dist

    for i in range(len(path)-1):
        total_len += dist(path[i], path[i+1], aoi2, aoi_threshold2, aoi_weight, violation_weight)
    
    if loop:
        total_len += dist(path[-1], path[0], aoi2, aoi_threshold2, aoi_weight, violation_weight)

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
                 min_round_trips=10,      # minimum number of round trips before stopping
                 max_round_trips=100,     # maximum number of round trips before stopping                 
                 min_ants=0,              # Total number of ants to use
                 max_ants=0,              # Total number of ants to use
                 
                 ant_count=32,            # 64 is the bottom of the near-optimal range for numpy performance
                 ant_speed=1,             # how many steps do ants travel per epoch

                 distance_power=0,        # power to which distance affects pheromones                 
                 pheromone_power=1.25,    # power to which differences in pheromones are noticed
                 decay_power=0,           # how fast do pheromones decay
                 reward_power=1,          # relative pheromone reward based on best_path_length/path_length 
                 best_path_smell=2,       # queen multiplier for pheromones upon finding a new best path                  
                 start_smell=0,           # amount of starting pheromones [0 defaults to `10**self.distance_power`]

                 verbose=False,

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
        
        if self.min_round_trips and self.max_round_trips:
            self.min_round_trips = min(self.min_round_trips, self.max_round_trips)
        if self.min_ants and self.max_ants:
            self.min_ants = min(self.min_ants, self.max_ants)


    def solve_initialize(
            self,
            problem_path: List[Any],
    ) -> None:

        ### Cache of distances between nodes
        self.distances = {
            source: {
                dest: self.cost_fn(source, dest)
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
              restart=False,
    ) -> List[Tuple[int,int]]:
    
        if restart or not self._initalized:
            self.solve_initialize(problem_path)

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
            
        
def AntColonyRunner(points, verbose=False, plot=False, label={}, algorithm=AntColonySolver, **kwargs):
    solver     = algorithm(cost_fn=tuple_dist, verbose=verbose, **kwargs)
    start_time = time.perf_counter()
    result     = solver.solve(points)
    stop_time  = time.perf_counter()
    if label: kwargs = { **label, **kwargs }
        
    for key in ['verbose', 'plot', 'animate', 'label', 'min_time', 'max_time']:
        if key in kwargs: del kwargs[key]
    print("N={:<3d} | {:5.0f} -> {:4.0f} | {:4.0f}s | ants: {:5d} | trips: {:4d} | "
          .format(len(points), path_lenght(points), path_lenght(result), (stop_time - start_time), solver.ants_used, solver.round_trips)
          + " ".join([ f"{k}={v}" for k,v in kwargs.items() ])
    )
    return result


def preprocess_set(Points : List[Any], drone_pos : Point, dist_fn=tuple_dist):

    ### Convert points to tuples
    new_problem = []
    for point in Points:
        new_problem.append(from_point_to_tuple(point))

    ### pfind the closest point to the current position of the drone
    p0 = from_point_to_tuple(drone_pos)
    closest_i = 0
    min_dist = float("inf")
    for i in range(len(new_problem)):
        dist = dist_fn(p0,new_problem[i])
        if dist < min_dist:
            closest_i = i
            min_dist = dist
    
    ### put the closest point as the start of the sequence
    tmp = new_problem[0]
    new_problem[0] = new_problem[closest_i]
    new_problem[closest_i] = tmp

    ### return preprocessed problem
    return new_problem

def postprocess_path(Path : List[Any], path_len):
    new_path = []
    for point in Path:
        new_path.append(from_tuple_to_point(point))
    return new_path[:path_len]



# ====================================================== TEST 

n_points = 2
path_len = int(math.sqrt(n_points)+1) * 2
Env = [Point() for x in range(n_points)]
aois = [random.uniform(0,9) for x in range(n_points)]
thresholds = [random.uniform(0,9) for x in range(n_points)]
aoi_weight = 1.0
violation_weight = 1.0

drone_pos = Point(0.0,0.0,0.0)
Points = preprocess_set(Env, drone_pos)
result = AntColonyRunner(Points, distance_power=1, verbose=False, plot=True)
path = postprocess_path(result, min(n_points,path_len))
print(path)