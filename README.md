# Taller 4 - Planificacion Automatizada

## Integrantes

| Nombre | Codigo |
|---|---|
| Daniel Vergara | 202320392 |
| Mariana Rodriguez | 202421258 |
| Martin de Angulo | 202421628 |
| Yara Gutierrez | 202511181 |

## Contexto del proyecto

El objetivo es implementar un sistema de planificacion automatizada para la "Operacion Fenix", donde un robot SAR debe rescatar pacientes en una cuadricula.

El robot puede moverse por el mapa, recoger suministros medicos, preparar un puesto medico, transportar pacientes y ejecutar la accion de rescate. El problema se modela con una representacion tipo PDDL, usando fluentes para describir el estado del mundo y acciones con precondiciones y efectos.

## Estructura general

- `PhoenixOperation/main.py`: punto de entrada del programa.
- `PhoenixOperation/planning/domain.py`: definicion de acciones del dominio.
- `PhoenixOperation/planning/pddl.py`: aplicabilidad de acciones, aplicacion de acciones y grounding.
- `PhoenixOperation/planning/problems.py`: definicion de `SimpleRescueProblem` y `MultiRescueProblem`.
- `PhoenixOperation/planning/planner.py`: planificadores clasicos (`forwardBFS`, `forwardSearch`, `backwardSearch`, `aStarPlanner`).
- `PhoenixOperation/planning/heuristics.py`: heuristicas para A*.
- `PhoenixOperation/planning/htn.py`: planificador jerarquico HTN.
- `PhoenixOperation/layouts/`: mapas de prueba.

## Como ejecutar

Desde la carpeta `PhoenixOperation`:

```powershell
python main.py -p SimpleRescueProblem -f forwardSearch -l tinyBase -q
```

Ejemplos utiles:

```powershell
python main.py -p SimpleRescueProblem -f tinyBaseSearch -l tinyBase -q
python main.py -p SimpleRescueProblem -f forwardSearch -l tinyBase -q
python main.py -p SimpleRescueProblem -f backwardSearch -l tinyBase -q
python main.py -p SimpleRescueProblem -f aStarPlanner -h ignoreDeleteLists -l tinyBase -q
python main.py -p SimpleRescueProblem -m -l htnBase -q
python main.py -p MultiRescueProblem -m -l tinyMulti -q
```

El flag `-q` ejecuta sin interfaz grafica. Para ver una visualizacion, se puede omitir `-q` o usar `-t` para modo texto.

## Algoritmos implementados

- Modelado del dominio con acciones tipo PDDL.
- Verificacion de acciones aplicables y aplicacion de efectos.
- Busqueda hacia adelante con BFS.
- Busqueda regresiva desde la meta.
- A* con heuristicas:
  - Ignorar precondiciones.
  - Ignorar listas de borrado.
- Planificacion jerarquica HTN con tareas de alto nivel para preparar suministros y rescatar pacientes.

## Limitaciones conocidas

- `backwardSearch` funciona en layouts pequenos como `tinyBase`, pero puede no encontrar plan en mapas mas grandes como `smallRescue`. Esto se debe al crecimiento del espacio de subobjetivos en la busqueda regresiva.
- A* puede reducir la cantidad de estados expandidos, pero las heuristicas implementadas son costosas de calcular. En algunos mapas, A* puede tardar mas que BFS.
- La implementacion HTN esta orientada al dominio del taller y usa una jerarquia simple de rescate. Funciona bien en los layouts probados, pero no garantiza encontrar el plan optimo global.
- Los mapas multi-rescate pueden ser mucho mas costosos para los planificadores clasicos, especialmente con BFS.

