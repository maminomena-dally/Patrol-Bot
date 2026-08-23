"""
security/speaker.py — À IMPLÉMENTER par le binôme Sécurité.

Rôle attendu (section 14 du cahier des charges) :
    Représenter le haut-parleur / sirène du robot. En simulation hors
    ligne, il n'est pas nécessaire de produire un vrai son : un état
    ON/OFF avec un type de son, journalisé, suffit.

    État normal :     speaker = OFF
    Intrusion :        speaker = ON, type_son = "ALERTE_INTRUSION"
    Fin d'événement :  speaker = OFF

Interface attendue :
    - `play(sound_type)` : passe l'état à ON avec le type de son donné.
    - `stop()` : repasse l'état à OFF.
    - `is_on` : propriété booléenne consultable par l'UI / les logs.

Exemple de squelette :

    class Speaker:
        def __init__(self):
            self.is_on = False
            self.sound_type = None

        def play(self, sound_type="ALERTE_INTRUSION"):
            self.is_on = True
            self.sound_type = sound_type

        def stop(self):
            self.is_on = False
            self.sound_type = None
"""

# TODO(binôme sécurité) : implémenter la classe Speaker (squelette simple
# ci-dessus, à enrichir si besoin d'une vraie sortie audio).
