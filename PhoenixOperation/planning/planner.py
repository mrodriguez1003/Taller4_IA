from __future__ import annotations

from collections.abc import Callable

from planning.pddl import (
    Action,
    ActionSchema,
    Problem,
    State,
    Objects,
    get_all_groundings,
)
from planning.utils import Queue, PriorityQueue
from planning.heuristics import nullHeuristic


# ---------------------------------------------------------------------------
# Reference implementation – read and understand before coding the rest.
# ---------------------------------------------------------------------------


def tinyBaseSearch(problem: Problem) -> list[Action]:
    """
    Hardcoded plan for the tinyBase layout.
    The robot at (1,4) must: pick up supplies at (1,3), set them up at (1,2),
    pick up the patient at (1,1), bring them to (1,2), and execute Rescue.

    Useful to understand the Action object format and plan structure.
    """
    robot = "robot"
    supplies = "supplies_0"
    patient = "patient_0"

    c14 = (1, 4)  # robot start
    c13 = (1, 3)  # supplies
    c12 = (1, 2)  # medical post
    c11 = (1, 1)  # patient

    plan = [
        Action(
            "Move(robot,(1,4),(1,3))",
            [("At", robot, c14), ("Adjacent", c14, c13), ("Free", c13)],
            [],
            [("At", robot, c13), ("Free", c14)],
            [("At", robot, c14), ("Free", c13)],
        ),
        Action(
            "PickUp(robot,supplies_0,(1,3))",
            [
                ("At", robot, c13),
                ("At", supplies, c13),
                ("HandsFree", robot),
                ("Pickable", supplies),
            ],
            [],
            [("Holding", robot, supplies)],
            [("At", supplies, c13), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,3),(1,2))",
            [("At", robot, c13), ("Adjacent", c13, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c13)],
            [("At", robot, c13), ("Free", c12)],
        ),
        Action(
            "SetupSupplies(robot,supplies_0,(1,2))",
            [("At", robot, c12), ("MedicalPost", c12), ("Holding", robot, supplies)],
            [("SuppliesReady", c12)],
            [("SuppliesReady", c12), ("HandsFree", robot)],
            [("Holding", robot, supplies)],
        ),
        Action(
            "Move(robot,(1,2),(1,1))",
            [("At", robot, c12), ("Adjacent", c12, c11), ("Free", c11)],
            [],
            [("At", robot, c11), ("Free", c12)],
            [("At", robot, c12), ("Free", c11)],
        ),
        Action(
            "PickUp(robot,patient_0,(1,1))",
            [
                ("At", robot, c11),
                ("At", patient, c11),
                ("HandsFree", robot),
                ("Pickable", patient),
            ],
            [],
            [("Holding", robot, patient)],
            [("At", patient, c11), ("HandsFree", robot)],
        ),
        Action(
            "Move(robot,(1,1),(1,2))",
            [("At", robot, c11), ("Adjacent", c11, c12), ("Free", c12)],
            [],
            [("At", robot, c12), ("Free", c11)],
            [("At", robot, c11), ("Free", c12)],
        ),
        Action(
            "PutDown(robot,patient_0,(1,2))",
            [("At", robot, c12), ("Holding", robot, patient)],
            [],
            [("At", patient, c12), ("HandsFree", robot)],
            [("Holding", robot, patient)],
        ),
        Action(
            "Rescue(robot,patient_0,(1,2))",
            [
                ("At", robot, c12),
                ("At", patient, c12),
                ("MedicalPost", c12),
                ("SuppliesReady", c12),
            ],
            [],
            [("Rescued", patient)],
            [("At", patient, c12)],
        ),
    ]
    return plan


# ---------------------------------------------------------------------------
# Punto 2 – Forward Planning
# ---------------------------------------------------------------------------


def forwardBFS(problem: Problem) -> list[Action]:
    """
    Forward BFS in state space.

    Explore states reachable from the initial state by applying actions,
    in breadth-first order, until a goal state is found.

    Returns a list of Action objects forming a valid plan, or [] if no plan exists.

    Tip: The state is a frozenset of fluents. Use problem.getSuccessors(state)
         to get (next_state, action, cost) triples. Track visited states to
         avoid revisiting the same state twice (graph search, not tree search).
    """
    ### Your code here ###
    start_state = problem.getStartState()

    if problem.isGoalState(start_state):
        return []

    frontier = Queue()
    frontier.push((start_state, []))
    visited = {start_state}

    while not frontier.isEmpty():
        state, plan = frontier.pop()

        for successor in problem.getSuccessors(state):
            next_state = successor[0]
            action = successor[1]

            if next_state in visited:
                continue

            next_plan = plan + [action]
            if problem.isGoalState(next_state):
                return next_plan

            visited.add(next_state)
            frontier.push((next_state, next_plan))

    return []
    ### End of your code ###


# ---------------------------------------------------------------------------
# Punto 3 – Backward Planning
# ---------------------------------------------------------------------------


def regress(goal_set: State, action: Action) -> State | None:
    """
    Compute the regression of goal_set through action.

    Given a goal description (set of fluents that must be true) and an action,
    return the new goal description that, if satisfied, guarantees the original
    goal is satisfied after executing action.

    REGRESS(g, a) = (g − ADD(a)) ∪ PRECOND_pos(a)
        IF:  ADD(a) ∩ g ≠ ∅   (action is relevant: contributes to the goal)
        AND: DEL(a) ∩ g = ∅   (action does not undo any goal fluent)
    Returns None if the action is not relevant or creates a contradiction.

    Tip: Use frozenset operations: intersection (&), difference (-), union (|).
         Check relevance first, then check for contradictions, then compute.
    """
    ### Your code here ###
    #Probar relevancia
    if not (action.add_list & goal_set):
        return None
    #Probar que no haya contradicciones
    if action.del_list & goal_set:
        return None
    #Regresión:
    return (goal_set - action.add_list) | action.precond_pos

    ### End of your code ###


# def backwardSearch(problem: Problem) -> list[Action]:
#     """
#     Backward search (regression search) from the goal.
# 
#     Start from the goal description and apply action regressions until
#     the resulting goal is satisfied by the initial state.
# 
#     Returns a list of Action objects forming a valid plan (in forward order),
#     or [] if no plan exists.
# 
#     Tip: The "state" in backward search is a frozenset of fluents that must
#          be true (a partial goal description). The initial state is reached
#          when all fluents in the current goal are satisfied by problem.initial_state.
#          Only consider actions whose add_list has at least one unsatisfied goal fluent
#          (relevant actions). Use regress() to compute the new subgoal.
#          Skip subgoals that contain static predicates (MedicalPost, Adjacent,
#          Pickable) that are false in the initial state — these are dead ends.
#     """
#     ### Your code here ###
#     
#     initial_state = problem.initial_state
#     goal= problem.goal
#     
#     #Predicados estaticos
#     static_predicates= {"MedicalPost", "Adjacent", "Pickable", "Free"}
#     
#     #Probar si la meta ya se cumple
#     if goal.issubset(initial_state):
#         return []
#     
#     #Obtner de una vez todas las acciones 
#     all_actions= get_all_groundings(problem.domain, problem.objects)
#     
#     #Filtrar
#     frontier = Queue()
#     frontier.push((goal, []))
#     visited = {goal}
#     MAX_VISITED= 50000 #Limite para evitar loops infinitos en casos sin solución
#     
#     while not frontier.isEmpty():
#         if len (visited)> MAX_VISITED:
#             break
#         current_goal, forward_plan = frontier.pop()
#         
#         #Solo considerar las acciones relevantes
#         for action in all_actions:
#             if not (action.add_list & current_goal):
#                 continue
#             
#             regressed= regress(current_goal, action)
#             if regressed is None:
#                 continue
#             
#             #Descartar metas con predicados estaticos falsos en el estado inicial
#             dead_end = any(
#                 f[0] in static_predicates and f not in initial_state
#                 for f in regressed
#             )
#             if dead_end:
#                 continue
#             
#             #No permitir que la meta se vuelva más grande (evitar loops)
#             if len(regressed) > len(initial_state):
#                 continue
#             
#             new_plan = [action] + forward_plan
#             if regressed.issubset(initial_state):
#                 return new_plan
#             if regressed not in visited:
#                 visited.add(regressed)
#                 frontier.push((regressed, new_plan))
#     return []
#                     
# 
#     ### End of your code ###
# 
# 
# ---------------------------------------------------------------------------
# Punto 4 – A* Planner
# ---------------------------------------------------------------------------
# 

def backwardSearch(problem: Problem) -> list[Action]:
    initial_state = problem.initial_state
    goal = problem.goal

    if goal.issubset(initial_state):
        return []

    all_actions = get_all_groundings(problem.domain, problem.objects)
    static_predicates = {"MedicalPost", "Adjacent", "Pickable"}

    frontier = Queue()
    frontier.push((goal, []))
    visited = {goal}
    max_regressed_goals = 50000

    while not frontier.isEmpty():
        if len(visited) > max_regressed_goals:
            return []

        current_goal, forward_plan = frontier.pop()
        unsatisfied_goal = current_goal - initial_state

        for action in all_actions:
            if not (action.add_list & unsatisfied_goal):
                continue

            regressed = regress(current_goal, action)
            if regressed is None or regressed in visited:
                continue

            if _has_false_static_fluent(regressed, initial_state, static_predicates):
                continue
            if _has_conflicting_positive_fluents(regressed):
                continue

            new_plan = [action] + forward_plan
            if regressed.issubset(initial_state):
                return new_plan

            visited.add(regressed)
            frontier.push((regressed, new_plan))

    return []


def _has_false_static_fluent(
    goal_set: State,
    initial_state: State,
    static_predicates: set[str],
) -> bool:
    return any(
        fluent[0] in static_predicates and fluent not in initial_state
        for fluent in goal_set
    )


def _has_conflicting_positive_fluents(goal_set: State) -> bool:
    at_locations: dict[object, object] = {}
    holding_objects: set[object] = set()
    hands_free = False

    for fluent in goal_set:
        predicate = fluent[0]
        if predicate == "At":
            entity, location = fluent[1], fluent[2]
            if entity in at_locations and at_locations[entity] != location:
                return True
            at_locations[entity] = location
        elif predicate == "Holding":
            holding_objects.add(fluent[2])
        elif predicate == "HandsFree":
            hands_free = True

    return hands_free and bool(holding_objects)

Heuristic = Callable[[State, State, list[ActionSchema], Objects], float]


def aStarPlanner(
    problem: Problem,
    heuristic: Heuristic = nullHeuristic,
) -> list[Action]:
    """
    Forward A* search guided by a heuristic.

    Combines the real accumulated cost g(n) with the heuristic estimate h(n)
    to prioritize which state to expand next: f(n) = g(n) + h(n).

    Returns a list of Action objects forming a valid plan, or [] if no plan exists.

    Tip: The heuristic signature is heuristic(state, goal, domain, objects) → float.
         Use PriorityQueue with priority = g + h(next_state).
         Track the best g-cost seen for each state to avoid stale expansions.
    """
    ### Your code here ###
    
    start_state = problem.getStartState()
    
    if problem.isGoalState(start_state):
        return []
    
    h0= heuristic(start_state, problem.goal, problem.domain, problem.objects)
    frontier = PriorityQueue()
    frontier.push((start_state, []), h0)
    
    #mejor costo encontrado para cada estado
    best_g: dict = { start_state:0}
    while not frontier.isEmpty():
        state, plan= frontier.pop()
        g= len(plan)
        
        #Stale expansion check
        if g> best_g.get(state, float('inf')):
            continue
        
        if problem.isGoalState(state):
            return plan
        
        for next_state, action, cost in problem.getSuccessors(state):
            new_g= g + cost
            if new_g < best_g.get(next_state, float('inf')):
                best_g[next_state] = new_g
                h= heuristic(next_state, problem.goal, problem.domain, problem.objects)
                priority= new_g + h
                frontier.push((next_state, plan + [action]), priority)
    return []

    ### End of your code ###


# Aliases used by the command-line argument parser
tinyBaseSearch = tinyBaseSearch
forwardBFS = forwardBFS
forwardSearch = forwardBFS
backwardSearch = backwardSearch
aStarPlanner = aStarPlanner
