import gym
import gym_grid_driving
from gym_grid_driving.envs.grid_driving import LaneSpec, Point
import os
import sys

FAST_DOWNWARD_DIRECTORY_ABSOLUTE_PATH = "/fast_downward/"
PDDL_FILE_ABSOLUTE_PATH = ""


class GeneratePDDL_Stationary:
    '''
    Class to generate the PDDL files given the environment description.
    '''

    def __init__(self, env, num_lanes, width, file_name):
        self.state = env.reset()
        self.num_lanes = num_lanes
        self.width = width
        self.file_name = file_name
        self.problem_file_name = self.file_name + 'problem.pddl'
        self.domain_file_name = self.file_name + 'domain.pddl'
        self.domain_string = ""
        self.type_string = ""
        self.predicate_strings = self.addHeader("predicates")
        self.action_strings = ""
        self.problem_string = ""
        self.object_strings = self.addHeader("objects")

    def addDomainHeader(self, name='default_header'):
        '''
        Adds the header in the domain file.

        Parameters :
        name (string): domain name.
        '''
        self.domain_header = "(define (domain " + name + " ) \n" + "(:requirements :strips :typing) \n"

    def addTypes(self, types={}):
        '''
        Adds the object types to the PDDL domain file.

        Parameters :
        types (dict): contains a dictionary of (k,v) pairs, where k is the object type, and v is the supertype. If k has no supertype, v is None.
        '''
        type_string = "(:types "

        for _type, _supertype in types.items():
            if _supertype is None:
                type_string += _type + "\n"
            else:
                type_string += _type + " - " + _supertype + "\n"
        type_string += ") \n"
        self.type_string = type_string

    def addPredicate(self, name='default_predicate', parameters=(), isLastPredicate=False):
        '''
        Adds predicates to the PDDL domain file

        Parameters :
        name (string) : name of the predicate.
        parameters (tuple or list): contains a list of (var_name, var_type) pairs, where var_name is an instance of object type var_type.
        isLastPredicate (bool) : True for the last predicate added.
        '''
        predicate_string = "(" + name
        for var_name, var_type in parameters:
            predicate_string += " ?" + var_name + " - " + var_type
        predicate_string += ") \n"
        self.predicate_strings += predicate_string

        if isLastPredicate:
            self.predicate_strings += self.addFooter()

    def addAction(self, name='default_action', parameters=(), precondition_string="", effect_string=""):
        '''
        Adds actions to the PDDL domain file

        Parameters :
        name (string) : name of the action.
        parameters (tuple or list): contains a list of (var_name, var_type) pairs, where var_name is an instance of object type var_type.
        precondition_string (string) : The precondition for the action.
        effect_string (string) : The effect of the action.
        '''
        action_string = name + "\n"
        parameter_string = ":parameters ("
        for var_name, var_type in parameters:
            parameter_string += " ?" + var_name + " - " + var_type
        parameter_string += ") \n"

        precondition_string = ":precondition " + precondition_string + "\n"
        effect_string = ":effect " + effect_string + "\n"
        action_string += parameter_string + precondition_string + effect_string
        action_string = self.addHeader("action") + action_string + self.addFooter()
        self.action_strings += action_string

    def generateDomainPDDL(self):
        '''
        Generates the PDDL domain file after all actions, predicates and types are added
        '''
        domain_file = open(PDDL_FILE_ABSOLUTE_PATH + self.domain_file_name, "w")
        PDDL_String = self.domain_header + self.type_string + self.predicate_strings + self.action_strings + self.addFooter()
        domain_file.write(PDDL_String)
        domain_file.close()

    def addProblemHeader(self, problem_name='default_problem_name', domain_name='default_domain_name'):
        '''
        Adds the header in the problem file.

        Parameters :
        problem_name (string): problem name.
        domain_name (string): domain name.
        '''
        self.problem_header = "(define (problem " + problem_name + ") \n (:domain " + domain_name + ") \n"

    def addObjects(self, obj_type, obj_list=[], isLastObject=False):
        '''
        Adds object instances of the same type to the problem file

        Parameters :
        obj_type (string) : the object type of the instances that are being added
        obj_list (list(str)) : a list of object instances to be added
        isLastObject (bool) : True for the last set of objects added.
        '''
        obj_string = ""
        for obj in obj_list:
            obj_string += obj + " "
        obj_string += " - " + obj_type
        self.object_strings += obj_string + "\n "
        if isLastObject:
            self.object_strings += self.addFooter()

    def addInitState(self):
        '''
        Generates the complete init state
        '''
        initString = self.generateInitString()
        self.initString = self.addHeader("init") + initString + self.addFooter()

    def addGoalState(self):
        '''
        Generates the complete goal state
        '''
        goalString = self.generateGoalString()
        self.goalString = self.addHeader("goal") + goalString + self.addFooter()

    def generateGridCells(self):
        '''
        Generates the grid cell objects.

        For a |X+1| x |Y+1| sized grid, |X+1| x |Y+1| objects to represent each grid cell are created.
        pt0pt0, pt1pt0, .... ptxpt0
        pt0pt1, pt1pt1, .... ptxpt1
        ..       ..            ..
        ..       ..            ..
        pt0pty, pt1pty, .... ptxpty


        '''
        self.grid_cell_list = []
        for w in range(self.width):
            for lane in range(self.num_lanes):
                self.grid_cell_list.append("pt{}pt{}".format(w, lane))

    def generateTimeSteps(self):
        self.time_steps = []
        for i in range(self.width):
            self.time_steps.append(f"step{i}")

    def get_up_next_position(self, current_x, current_y):
        next_x = current_x - 1
        next_y = current_y - 1
        if next_y < 0:
            next_y = 0
        if next_x < 0:
            next_x = 0
        return next_x, next_y

    def get_down_next_position(self, current_x, current_y):
        next_x = current_x - 1
        next_y = current_y + 1
        if next_y >= self.num_lanes:
            next_y = self.num_lanes-1
        if next_x < 0:
            next_x = 0
        return next_x, next_y

    def get_forward_next_position(self, current_x, current_y, speed):
        next_x = current_x + speed[0]
        next_y = current_y
        if next_x < 0:
            next_x = 0
        return next_x, next_y

    def generateInitString(self):
        '''
        FILL ME : Should return the init string in the problem PDDL file.
        Hint : Use the defined grid cell objects from genearateGridCells and predicates to construct the init string.

        Information that might be useful here :

        1. Initial State of the environment : self.state
        2. Agent's x position : self.state.agent.position.x
        3. Agent's y position : self.state.agent.position.y
        4. The object of type agent is called "agent1" (see generateProblemPDDLFile() ).
        5. Set of cars in the grid: self.state.cars
        6. For a car in self.state.cars, it's x position: car.position.x
        7. For a car in self.state.cars, it's y position: car.position.y
        8. List of grid cell objects : self.grid_cell_list
        9. Width of the grid: self.width
        10. Number of lanes in the grid : self.num_lanes

        Play with environment (https://github.com/cs4246/gym-grid-driving) to see the type of values above objects return

        Example: The following statement adds the initial condition string from https://github.com/pellierd/pddl4j/blob/master/pddl/logistics/p01.pddl

        return "(at apn1 apt2) (at tru1 pos1) (at obj11 pos1) (at obj12 pos1) (at obj13 pos1) (at tru2 pos2) (at obj21 pos2) (at obj22 pos2)
                (at obj23 pos2) (in-city pos1 cit1) (in-city apt1 cit1) (in-city pos2 cit2) (in-city apt2 cit2)"
        '''
        init_string = ""

        # Get the state of car
        agent_grid_cell = self.grid_cell_list[self.num_lanes * self.state.agent.position.x +
                                              self.state.agent.position.y]
        agent_string = f"(at {agent_grid_cell} agent1 {self.time_steps[0]})"
        # Generate up_next, down_next, forward_next for each position in grid:
        for w in range(self.width) :
            for lane in range(self.num_lanes):
                if w > 0:
                    current_cell = self.grid_cell_list[self.num_lanes * w + lane]
                    forward_next_x, forward_next_y = self.get_forward_next_position(w, lane, [-1, -1])
                    next_grid_cell = self.grid_cell_list[self.num_lanes * forward_next_x + forward_next_y]
                    init_string += f"(forward1_next {current_cell} {next_grid_cell})"

                    up_next_x, up_next_y = self.get_up_next_position(w, lane,)
                    next_grid_cell = self.grid_cell_list[self.num_lanes * up_next_x + up_next_y]
                    init_string += f"(up_next {current_cell} {next_grid_cell})"

                    down_next_x, down_next_y = self.get_down_next_position(w, lane)
                    next_grid_cell = self.grid_cell_list[self.num_lanes * down_next_x + down_next_y]
                    init_string += f"(down_next {current_cell} {next_grid_cell})"
                if w > 1:
                    current_cell = self.grid_cell_list[self.num_lanes * w + lane]
                    forward_next_x, forward_next_y = self.get_forward_next_position(w, lane, [-2, -2])
                    next_grid_cell = self.grid_cell_list[self.num_lanes * forward_next_x + forward_next_y]
                    init_string += f"(forward2_next {current_cell} {next_grid_cell})"
                if w > 2:
                    current_cell = self.grid_cell_list[self.num_lanes * w + lane]
                    forward_next_x, forward_next_y = self.get_forward_next_position(w, lane, [-3, -3])
                    next_grid_cell = self.grid_cell_list[self.num_lanes * forward_next_x + forward_next_y]
                    init_string += f"(forward3_next {current_cell} {next_grid_cell})"

        # Get allowable time
        for i in range(len(self.time_steps)-1):
            init_string += f"(time_next {self.time_steps[i]} {self.time_steps[i+1]})"

        # Get block position
        car_string = ""
        car_set = set()
        for car in self.state.cars:
            x_pos = car.position.x
            y_pos = car.position.y
            car_grid_cell = self.grid_cell_list[self.num_lanes * x_pos +y_pos]
            car_set.add(f"(blocked {car_grid_cell} {self.time_steps[0]})")
            for i in range(1, len(self.time_steps)):
                for _ in range(abs(car.speed_range[0])):
                    x_pos = x_pos - 1
                    if x_pos < 0:
                        x_pos = self.width-1
                    y_pos = y_pos
                    car_grid_cell = self.grid_cell_list[self.num_lanes * x_pos + y_pos]
                    car_set.add(f"(blocked {car_grid_cell} {self.time_steps[i]})")

        for element in car_set:
            car_string += element

        init_string = init_string + agent_string + car_string

        return init_string

    def generateGoalString(self):
        '''
        FILL ME : Should return the goal string in the problem PDDL file
        Hint : Use the defined grid cell objects from genearateGridCells and predicates to construct the goal string.

        Information that might be useful here :
        1. Goal x Position : self.state.finish_position.x
        2. Goal y Position : self.state.finish_position.y
        3. The object of type agent is called "agent1" (see generateProblemPDDLFile() ).
        Play with environment (https://github.com/cs4246/gym-grid-driving) to see the type of values above objects return

        Example: The following statement adds goal string from https://github.com/pellierd/pddl4j/blob/master/pddl/logistics/p01.pddl

        return "(and (at obj11 apt1) (at obj23 pos1) (at obj13 apt1) (at obj21 pos1)))"
        '''
        goal_grid_cell = self.grid_cell_list[self.num_lanes * self.state.finish_position.x +
                                              self.state.finish_position.y]

        goal_string = "(or "
        for i in range(self.width): # reach goal at any steps
            goal_string += f" (at {goal_grid_cell} agent1 step{i})"
        return goal_string + ")"

    def generateProblemPDDL(self):
        '''
        Generates the PDDL problem file after the object instances, init state and goal state are added
        '''
        problem_file = open(PDDL_FILE_ABSOLUTE_PATH + self.problem_file_name, "w")
        PDDL_String = self.problem_header + self.object_strings + self.initString + self.goalString + self.addFooter()
        problem_file.write(PDDL_String)
        problem_file.close()

    '''
    Helper Functions 
    '''

    def addHeader(self, name):
        return "(:" + name + " "

    def addFooter(self):
        return ") \n"


def initializeSystem(env):
    gen = GeneratePDDL_Stationary(env, len(env.lanes), width=env.width, file_name='HW1')
    return gen


def generateDomainPDDLFile(gen):
    '''
    Function that specifies the domain and generates the PDDL Domain File.
    As a part of the assignemnt, you will need to add the actions here.
    '''
    gen.addDomainHeader("grid_world")
    gen.addTypes(types={"car": None, "agent": "car", "gridcell": None, "time": None})
    '''
    Predicate Definitions :
    (at ?pt ?car ?t) : car is at gridcell pt at time ?t
    (up_next ?pt1 ?pt2) : pt2 is the next location of the car when it takes the UP action from pt1
    (down_next ?pt1 ?pt2) : pt2 is the next location of the car when it takes the DOWN action from pt1
    (time_next ?t1 ?t2): t2 is the next time step of t1
    (forward_next1 ?pt1 ?pt2) : pt2 is the next location of the car when it takes the FORWARD1 action from pt1
    (forward_next2 ?pt1 ?pt2) : pt2 is the next location of the car when it takes the FORWARD2 action from pt1
    (forward_next3 ?pt1 ?pt2) : pt2 is the next location of the car when it takes the FORWARD3 action from pt1
    (blocked ?pt) : The gridcell pt is occupied by a car and is "blocked".
    '''
    gen.addPredicate(name="at", parameters=(("pt1", "gridcell"), ("car", "car"), ("t1", "time")))
    gen.addPredicate(name="up_next", parameters=(("pt1" , "gridcell"), ("pt2", "gridcell")))
    gen.addPredicate(name="time_next", parameters=(("t1", "time"), ("t2", "time")))
    gen.addPredicate(name="down_next", parameters=(("pt1" , "gridcell"), ("pt2", "gridcell")))
    gen.addPredicate(name="forward1_next", parameters=(("pt1" , "gridcell"), ("pt2", "gridcell")))
    gen.addPredicate(name="forward2_next", parameters=(("pt1" , "gridcell"), ("pt2", "gridcell")))
    gen.addPredicate(name="forward3_next", parameters=(("pt1" , "gridcell"), ("pt2", "gridcell")))
    gen.addPredicate(name="blocked", parameters=[("pt1" , "gridcell"), ("t1", "time")] , isLastPredicate=True)
    '''
    FILL ME : Add the actions UP, DOWN, FORWARD with the help of gen.addAction() as follows :

        gen.addAction(name="UP", parameters = (...), precondition_string = "...", effect_string="...")
        gen.addAction(name="DOWN", parameters = (...), precondition_string = "...", effect_string="...")
        gen.addAction(name="FORWARD", parameters = (...), precondition_string = "...", effect_string="...")

        You have to fill up the ... in each of gen.addAction() above.

    Example :

    The following statement adds the LOAD-TRUCK action from https://tinyurl.com/y3jocxdu [The domain file referenced in the assignment] to the domain file 
    gen.addAction(name="LOAD-TRUCK", 
                  parameters=(("pkg", "package"), ("truck" , "truck"), ("loc", "place")), 
                  precondition_string="(and (at ?truck ?loc) (at ?pkg ?loc))", 
                  effect_string= "(and (not (at ?pkg ?loc)) (in ?pkg ?truck))")
    '''
    gen.addAction(name="UP",
                  parameters=(("car", "agent"), ("pt1", "gridcell"), ("pt2", "gridcell"), ("t1", "time"), ("t2", "time")),
                  precondition_string="(and (at ?pt1 ?car ?t1) (time_next ?t1 ?t2)  (up_next ?pt1 ?pt2) (not (blocked ?pt2 ?t2)))",
                  effect_string="(and (not (at ?pt1 ?car ?t1)) (at ?pt2 ?car ?t2))"
                  )
    gen.addAction(name = "DOWN",
                  parameters=(("car", "agent"), ("pt1", "gridcell"), ("pt2", "gridcell"), ("t1", "time"), ("t2", "time")),
                  precondition_string="(and (at ?pt1 ?car ?t1) (time_next ?t1 ?t2) (down_next ?pt1 ?pt2) (not (blocked ?pt2 ?t2)))",
                  effect_string="(and (not (at ?pt1 ?car ?t1)) (at ?pt2 ?car ?t2))"
                  )
    gen.addAction(name = "FORWARD3",
                  parameters=(("car", "agent"), ("pt1", "gridcell"), ("pt2", "gridcell"), ("t1", "time"), ("t2", "time")),
                  precondition_string="(and (at ?pt1 ?car ?t1) (time_next ?t1 ?t2) (forward3_next ?pt1 ?pt2) (not (blocked ?pt2 ?t2)))",
                  effect_string="(and (not (at ?pt1 ?car ?t1)) (at ?pt2 ?car ?t2))"
                  )
    gen.addAction(name = "FORWARD2",
                  parameters=(("car", "agent"), ("pt1", "gridcell"), ("pt2", "gridcell"), ("t1", "time"), ("t2", "time")),
                  precondition_string="(and (at ?pt1 ?car ?t1) (time_next ?t1 ?t2) (forward2_next ?pt1 ?pt2) (not (blocked ?pt2 ?t2)))",
                  effect_string="(and (not (at ?pt1 ?car ?t1)) (at ?pt2 ?car ?t2))"
                  )
    gen.addAction(name = "FORWARD1",
                  parameters=(("car", "agent"), ("pt1", "gridcell"), ("pt2", "gridcell"), ("t1", "time"), ("t2", "time")),
                  precondition_string="(and (at ?pt1 ?car ?t1) (time_next ?t1 ?t2) (forward1_next ?pt1 ?pt2) (not (blocked ?pt2 ?t2)))",
                  effect_string="(and (not (at ?pt1 ?car ?t1)) (at ?pt2 ?car ?t2))"
                  )

    gen.generateDomainPDDL()
    pass


def generateProblemPDDLFile(gen):
    '''
    Function that specifies the domain and generates the PDDL Domain File.
    Objects defined here should be used to construct the init and goal strings
    '''
    gen.addProblemHeader("parking", "grid_world")
    gen.addObjects("agent", ["agent1"])
    gen.generateTimeSteps()
    gen.addObjects("time", gen.time_steps)
    gen.generateGridCells()
    gen.addObjects("gridcell", gen.grid_cell_list, isLastObject=True)
    gen.addInitState()
    gen.addGoalState()
    gen.generateProblemPDDL()
    pass


def runPDDLSolver(gen):
    '''
    Runs the fast downward solver to get the optimal plan
    '''
    print(FAST_DOWNWARD_DIRECTORY_ABSOLUTE_PATH)
    os.system(
        FAST_DOWNWARD_DIRECTORY_ABSOLUTE_PATH + 'fast-downward.py ' + PDDL_FILE_ABSOLUTE_PATH + gen.domain_file_name + ' ' + PDDL_FILE_ABSOLUTE_PATH + gen.problem_file_name + ' --search  \"lazy_greedy([ff()], preferred=[ff()])\"' + ' > temp ')


def delete_files(gen):
    '''
    Deletes PDDL and plan files created.
    '''
    os.remove(PDDL_FILE_ABSOLUTE_PATH + gen.domain_file_name)
    os.remove(PDDL_FILE_ABSOLUTE_PATH + gen.problem_file_name)
    os.remove('sas_plan')


def simulateSolution(env):
    '''
    Simulates the plan given by the solver on the environment
    '''
    env.render()
    plan_file = open('sas_plan', 'r')
    for line in plan_file.readlines() :
        if line[0] == '(' :
            action = line.split()[0][1:]
            print(action)
            if action == 'up' :
                env.step(env.actions[0])
            if action == 'down' :
                env.step(env.actions[1])
            if action == 'forward3' :
                env.step(env.actions[2])
            if action == "forward2":
                env.step(env.actions[3])
            if action == "forward1":
                env.step(env.actions[4])
            env.render()

def generatePlan(env):
    '''
    Extracts the plan given by the solver into a list of actions
    '''
    plan_file = open('sas_plan', 'r')
    action_sequence = []
    for line in plan_file.readlines() :
        if line[0] == '(' :
            action = line.split()[0][1:]
            if action == 'up' :
                action_sequence.append(env.actions[0])
            if action == 'down' :
                action_sequence.append(env.actions[1])
            if action == 'forward3' :
                action_sequence.append(env.actions[2])
            if action == 'forward2' :
                action_sequence.append(env.actions[3])
            if action == 'forward1' :
                action_sequence.append(env.actions[4])
    return action_sequence


def test():
    '''
    Generates the PDDL files, solves for the optimal solution and simulates the plan. The PDDL files are deleted at the end.
    '''

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('problem', type=str, choices=['parking', 'crossing'], help='problem name')
    parser.add_argument('testcase', type=int, help='test case number')
    args = parser.parse_args()

    test_configs = {}
    test_configs['parking'] = [{'lanes': [LaneSpec(2, [0, 0])] * 3, 'width': 5, 'seed': 13},
                               {'lanes': [LaneSpec(2, [0, 0])] * 3, 'width': 5, 'seed': 10},
                               {'lanes': [LaneSpec(3, [0, 0])] * 4, 'width': 10, 'seed': 25},
                               {'lanes': [LaneSpec(4, [0, 0])] * 4, 'width': 10, 'seed': 25},
                               {'lanes': [LaneSpec(8, [0, 0])] * 7, 'width': 20, 'seed': 25},
                               {'lanes': [LaneSpec(7, [0, 0])] * 10, 'width': 20, 'seed': 125}]

    test_configs['crossing'] = [{'lanes': [LaneSpec(6, [-2, -2])] * 2 + [LaneSpec(6, [-5, -5])] * 2 +
                                          [LaneSpec(5, [-4, -4])] * 2 + [LaneSpec(5, [-2, -2])] * 1, 'width': 30,
                                 'seed': 101}]

    test_config = test_configs[args.problem]
    test_case_number = args.testcase
    LANES = test_config[test_case_number]['lanes']
    WIDTH = test_config[test_case_number]['width']
    RANDOM_SEED = test_config[test_case_number]['seed']

    if args.problem == "parking":
        env = gym.make('GridDriving-v0', lanes=LANES, width=WIDTH, random_seed=RANDOM_SEED, agent_speed_range=(-1, -1))
    elif args.problem == "crossing":
        env = gym.make('GridDriving-v0', lanes=LANES, width=WIDTH, random_seed=RANDOM_SEED, agent_speed_range=(-3, -1))

    gen = initializeSystem(env)
    generateDomainPDDLFile(gen)
    generateProblemPDDLFile(gen)
    runPDDLSolver(gen)
    simulateSolution(env)
    # delete_files(gen)


try:
    from runner.abstracts import Agent
except:
    class Agent(object):
        pass


class PDDLAgent(Agent):
    def initialize(self, fast_downward_path, env):
        global FAST_DOWNWARD_DIRECTORY_ABSOLUTE_PATH
        FAST_DOWNWARD_DIRECTORY_ABSOLUTE_PATH = fast_downward_path
        self.env = env
        gen = initializeSystem(self.env)
        generateDomainPDDLFile(gen)
        generateProblemPDDLFile(gen)
        runPDDLSolver(gen)
        self.action_plan = generatePlan(self.env)
        self.time_step = 0
        delete_files(gen)

    def step(self, state, *args, **kwargs):
        action = self.action_plan[self.time_step]
        self.time_step += 1
        return action


def create_agent(test_case_env, *args, **kwargs):
    return PDDLAgent()


if __name__ == '__main__':
    test()
