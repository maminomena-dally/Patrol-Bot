"""
planning/rrt.py — À IMPLÉMENTER par le binôme Planification.

Rôle attendu (slide 24 du support de cours) :
    Construire un arbre exploratoire aléatoire (RRT) dans l'espace 2D
    continu de la carte, utile en complément d'A* pour des cartes moins
    structurées ou pour la replanification rapide après un obstacle
    imprévu (section 12 du cahier des charges).

Interface attendue :
    - `plan(free_space_check, start, goal, max_iter=1000) -> list[(x, y)]`
      où `free_space_check(x, y) -> bool` teste si un point est libre
      (à combiner avec robot.get_footprint() pour les collisions).

Exemple de squelette :

    def rrt(free_space_check, start, goal, max_iter=1000, step_size=0.3):
        # TODO: implémenter RRT (voir LaValle, Planning Algorithms, chap. 5)
        raise NotImplementedError
"""

# TODO(binôme planification) : implémenter rrt().
