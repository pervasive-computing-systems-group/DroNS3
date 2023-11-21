import commands
import missions

class EnergyBudget(object):
    def __init__(self, mission):
        self.mission = mission
      
    def percentBudget(self):
        self.used = self.mission.initial_budget - self.mission.energy_budget
        return self.used/self.mission.initial_budget