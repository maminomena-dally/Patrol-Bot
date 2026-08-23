"""
planning/astar.py — À IMPLÉMENTER par le binôme Planification.

Rôle attendu (slides 24-25 du support de cours) :
    Calculer un chemin sur une grille d'occupation en minimisant
    f(n) = g(n) + h(n), avec g le coût réel depuis le départ et h une
    heuristique (distance euclidienne ou Manhattan) qui ne surestime
    jamais le coût restant, pour conserver l'optimalité.

Interface attendue :
    - `plan(grid, start, goal) -> list[(x, y)]` : retourne une liste de
      points de passage (chemin), à transmettre ensuite à
      control/pure_pursuit.py pour le suivi de trajectoire.
    - `grid` : représentation à définir par ce binôme (ex: tableau 2D de
      0 (libre) / 1 (occupé)), cohérente avec robot.get_footprint() pour
      les tests de collision.

Exemple de squelette :

    def astar(grid, start, goal):
        # TODO: implémenter l'algorithme A* (voir LaValle, Planning
        # Algorithms, ou Dijkstra + heuristique admissible)
        raise NotImplementedError
"""

# TODO(binôme planification) : implémenter astar().
