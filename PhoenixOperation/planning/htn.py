from __future__ import annotations
# Previous code kept for traceability:
# from asyncio import Queue
from planning.utils import Queue

from planning.pddl import Action, Problem, apply_action, is_applicable


# ---------------------------------------------------------------------------
# HTN Infrastructure
# ---------------------------------------------------------------------------


class HLA:
    """
    A High-Level Action (HLA) in HTN planning.

    An HLA is an abstract task that can be refined into sequences of
    more primitive actions (or other HLAs). Each refinement is a list
    of HLA or Action objects.

    name:        Human-readable name for display
    refinements: List of possible refinements, each a list of HLA/Action objects
    """

    def __init__(self, name: str, refinements: list[list] | None = None) -> None:
        self.name = name
        self.refinements = refinements or []

    def __repr__(self) -> str:
        return f"HLA({self.name})"


def is_primitive(action: Action | HLA) -> bool:
    """Return True if action is a primitive (grounded Action), False if it is an HLA."""
    return isinstance(action, Action)


def is_plan_primitive(plan: list[Action | HLA]) -> bool:
    """Return True if every step in the plan is a primitive action."""
    return all(is_primitive(step) for step in plan)


# ---------------------------------------------------------------------------
# Punto 5a – hierarchicalSearch
# ---------------------------------------------------------------------------


def hierarchicalSearch(problem: Problem, hlas: list[HLA]) -> list[Action]:
    """
    HTN planning via BFS over hierarchical plan refinements.

    Start with an initial plan containing a single top-level HLA.
    At each step, find the first non-primitive step in the plan and
    replace it with one of its refinements. Continue until the plan
    is fully primitive and achieves the goal when executed from the
    initial state.

    Returns a list of primitive Action objects, or [] if no plan found.

    Tip: The search space consists of (partial plan, current plan index) pairs.
         Use a Queue (BFS) to explore all refinement choices fairly.
         A plan is a solution when:
           1. It contains only primitive actions (is_plan_primitive), AND
           2. Executing it from the initial state reaches a goal state.
         To simulate execution, apply each action in order using apply_action().
    """
    ### Your code here ###
    top_level_hla = hlas[0]
    initial_plan = [top_level_hla]
    frontier = Queue()
    frontier.push(initial_plan)
 
    while not frontier.isEmpty():
        plan = frontier.pop()
        first_hla_idx = None
        for i, step in enumerate(plan):
            if not is_primitive(step):
                first_hla_idx = i
                break
        if first_hla_idx is None:
            state = problem.initial_state
            valid = True
            for action in plan:
                if not is_applicable(state, action):
                    valid = False
                    break
                state = apply_action(state, action)
            if valid and problem.isGoalState(state):
                return plan
            continue
        hla_to_expand = plan[first_hla_idx]
        for refinement in hla_to_expand.refinements:
            new_plan = plan[:first_hla_idx] + refinement + plan[first_hla_idx + 1:]
            frontier.push(new_plan)
    return []
    ### End of your code ###


# ---------------------------------------------------------------------------
# Punto 5b – HLA Definitions
# ---------------------------------------------------------------------------


def build_htn_hierarchy(problem: Problem) -> list[HLA]:
    """
    Build HTN HLAs for the rescue domain.

    The hierarchy defines four HLA types:
      - Navigate(from, to):       Move the robot step by step from one cell to another
      - PrepareSupplies(s, m):    Collect supplies and set them up at the medical post
      - ExtractPatient(p, m):     Pick up the patient and bring them to the medical post
      - FullRescueMission(s,p,m): Complete one rescue: prepare supplies + extract + rescue

    Refinements are built from the ground state to generate concrete Action objects.

    Tip: Refinements for Navigate are all single-step Move sequences between
         adjacent cells. PrepareSupplies and ExtractPatient chain Navigate HLAs
         with primitive PickUp, SetupSupplies, PutDown, and Rescue actions.
    """
    ### Your code here ###
    #
    # ### Your code here ###
    #


    layout = problem.layout
    if not problem.objects["patients"]:
        return []
    if not problem.objects["supplies"] or not problem.objects["medical_posts"]:
        return []

    robot = problem.objects["robots"][0]
    medical_post = problem.objects["medical_posts"][0]
    supply = problem.objects["supplies"][0]

    positions = {
        fluent[1]: fluent[2]
        for fluent in problem.initial_state
        if fluent[0] == "At"
    }

    adjacent = {}
    for cell in layout.get_all_cells():
        adjacent[cell] = []
    for a, b in layout.get_adjacent_pairs():
        adjacent[a].append(b)
        adjacent[b].append(a)

    def find_path(start, goal):
        frontier = Queue()
        frontier.push(start)
        parent = {start: None}

        while not frontier.isEmpty():
            cell = frontier.pop()
            if cell == goal:
                break
            for next_cell in adjacent[cell]:
                if next_cell not in parent:
                    parent[next_cell] = cell
                    frontier.push(next_cell)

        if goal not in parent:
            return []

        path = []
        cell = goal
        while cell is not None:
            path.append(cell)
            cell = parent[cell]
        path.reverse()
        return path

    def move_actions(start, goal):
        path = find_path(start, goal)
        actions = []
        for from_cell, to_cell in zip(path, path[1:]):
            actions.append(
                Action(
                    f"Move({robot}, {from_cell}, {to_cell})",
                    [
                        ("At", robot, from_cell),
                        ("Adjacent", from_cell, to_cell),
                        ("Free", to_cell),
                    ],
                    [],
                    [("At", robot, to_cell), ("Free", from_cell)],
                    [("At", robot, from_cell), ("Free", to_cell)],
                )
            )
        return actions

    current = positions[robot]

    prepare_actions: list[Action] = []
    supply_loc = positions[supply]
    prepare_actions += move_actions(current, supply_loc)
    current = supply_loc
    prepare_actions.append(
        Action(
            f"PickUp({robot}, {supply}, {current})",
            [
                ("At", robot, current),
                ("At", supply, current),
                ("HandsFree", robot),
                ("Pickable", supply),
            ],
            [],
            [("Holding", robot, supply)],
            [("At", supply, current), ("HandsFree", robot)],
        )
    )
    prepare_actions += move_actions(current, medical_post)
    current = medical_post
    prepare_actions.append(
        Action(
            f"SetupSupplies({robot}, {supply}, {medical_post})",
            [
                ("At", robot, medical_post),
                ("Holding", robot, supply),
                ("MedicalPost", medical_post),
            ],
            [],
            [("SuppliesReady", medical_post), ("HandsFree", robot)],
            [("Holding", robot, supply)],
        )
    )

    prepare_supplies = HLA(
        f"PrepareSupplies({supply},{medical_post})",
        [prepare_actions],
    )

    mission_steps: list[HLA] = [prepare_supplies]
    for patient in problem.objects["patients"]:
        patient_loc = positions[patient]
        patient_actions: list[Action] = []
        patient_actions += move_actions(current, patient_loc)
        current = patient_loc
        patient_actions.append(
            Action(
                f"PickUp({robot}, {patient}, {current})",
                [
                    ("At", robot, current),
                    ("At", patient, current),
                    ("HandsFree", robot),
                    ("Pickable", patient),
                ],
                [],
                [("Holding", robot, patient)],
                [("At", patient, current), ("HandsFree", robot)],
            )
        )
        patient_actions += move_actions(current, medical_post)
        current = medical_post
        patient_actions.append(
            Action(
                f"PutDown({robot}, {patient}, {medical_post})",
                [("At", robot, medical_post), ("Holding", robot, patient)],
                [],
                [("At", patient, medical_post), ("HandsFree", robot)],
                [("Holding", robot, patient)],
            )
        )
        patient_actions.append(
            Action(
                f"Rescue({robot}, {patient}, {medical_post})",
                [
                    ("At", robot, medical_post),
                    ("At", patient, medical_post),
                    ("MedicalPost", medical_post),
                    ("SuppliesReady", medical_post),
                ],
                [],
                [("Rescued", patient)],
                [("At", patient, medical_post)],
            )
        )

        mission_steps.append(
            HLA(f"ExtractPatient({patient},{medical_post})", [patient_actions])
        )

    root = HLA("FullRescueMission", [mission_steps])
    return [root]
    ### End of your code ###
