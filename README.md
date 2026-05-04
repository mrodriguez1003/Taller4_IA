# Taller de Lógica — Investigación Criminalística

## Integrantes

| Nombre | Código |
|---|---|
| Daniel Vergara | 202320392 |
| Mariana Rodriguez | 202421258 |
| Martin de Angulo | 202421628 |
| Yara Gutierrez | 202511181 |

## Descripción del proyecto

Este proyecto corresponde al **Taller de Lógica — Investigación Criminalística**. Su objetivo es resolver casos criminales usando herramientas de **lógica proposicional** y **lógica de predicados**.

A partir de una base de conocimiento, reglas lógicas, sospechosos, evidencias y relaciones entre hechos, el sistema permite analizar distintos escenarios criminalísticos y determinar conclusiones válidas mediante técnicas de razonamiento automático.

El proyecto incluye implementación de algoritmos de inferencia lógica, transformación de fórmulas a forma normal conjuntiva y model checking.


## Estructura del proyecto

```text
Clue/
├── src/                              # Motor de lógica
│   ├── logic_core.py                 # AST de fórmulas proposicionales (dado)
│   ├── model_checking.py             # Implementación de model checking
│   ├── cnf_transform.py              # Transformación a CNF
│   ├── resolution.py                 # Resolución proposicional (dado)
│   ├── predicate_logic.py            # Lógica de predicados (dado)
│   ├── forward_chaining.py           # Forward chaining (dado)
│   ├── backward_chaining.py          # Backward chaining (dado)
│   ├── utils.py                      # Utilidades de visualización
│   ├── crime_case.py                 # Estructura de datos de casos
│   └── tui.py                        # Interfaz gráfica de terminal
├── crimes/                           # Casos criminales
│   ├── veneno_villa_espinas.py       # Asesinato con arsénico
│   ├── robo_expreso_sur.py           # Robo de joyas en tren
│   ├── sabotaje_pharmax.py           # Sabotaje a laboratorio
│   ├── herencia_hacienda_rosal.py    # Envenenamiento por herencia
│   └── red_puerto_sombras.py         # Red de contrabando
├── tests/                            # Pruebas unitarias
│   ├── test_model_checking.py        # Tests para Punto 1
│   ├── test_cnf.py                   # Tests para Punto 2
│   └── test_predicates.py            # Tests para Punto 3
├── notebooks/                        # Guías de aprendizaje
│   ├── guia_objetos_python.ipynb     # Guía de POO en Python
│   ├── parte1_model_checking.ipynb   # Guía: model checking
│   ├── parte2_cnf.ipynb              # Guía: transformación a CNF
│   └── parte3_predicados.ipynb       # Guía: lógica de predicados
├── main.py                           # Interfaz gráfica/opcional
└── pyproject.toml                    # Configuración del proyecto
