"""
Refers to a coalition of pursuers.
Contains - list of pursuers involved, the evader they are currently pursuing, a function, to calculate the interception point accordingly, and move all the pursuers towards it.
"""
class Coalition:
    def __init__(self, pursuers, evader=None):
        self.pursuers = pursuers
        self.evader = None
    def set_evader(self, evader):
        self.evader = evader
    