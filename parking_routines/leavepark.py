import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule

from vehicle_mechanics.control import Controller, Stage
from .model import FuzzyModel


class LeaveParkController(Controller):
    def __init__(self, tank):
        stages = [
            LastAdjustment(),
            ParkSecondTurn(),
            ParkFirstTurn(),
            DriveCloserSecondTurn(),
            DriveCloserFirstTurn(),
            ForwardToExit(),
            CompleteStop()  # New stage to ensure complete stop
        ]
        super().__init__(tank, stages)


class CompleteStop(Stage):
    def control(self, tank, distances):
        # Completely stop the tank
        tank.stop()
        return True  # Immediately return True to complete this stage


class LastAdjustment(Stage):
    diff = Antecedent(np.linspace(-2, 2, 200), 'diff')
    vel = Consequent(np.linspace(-4, 4, 200), 'vel')

    best = -0.5
    slope = 0.4
    speed = 1.5

    diff['l'] = fuzz.trapmf(diff.universe, [-2, -2, best - slope, best])
    diff['m'] = fuzz.trimf(diff.universe, [best - slope, best, best + slope])
    diff['h'] = fuzz.trapmf(diff.universe, [best, best + slope, 2, 2])

    vel['zero'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['forward'] = fuzz.trimf(vel.universe, [speed-1, speed, speed+1])
    vel['backward'] = fuzz.trimf(vel.universe, [-speed-1, -speed, -speed+1])

    rules = [
        Rule(diff['h'], vel['forward']),
        Rule(diff['l'], vel['backward']),
        Rule(diff['m'], vel['zero']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    @classmethod
    def get_velocity(cls, distances):
        front = min(distances.nw2, distances.ne2)
        back = min(distances.sw2, distances.se2)
        diff = front - back
        cls.simulation.input['diff'] = diff
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.1:
            return 0
        return -velocity  # Reverse the sign to move backwards

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.backward(abs(velocity))  # Use backward movement
        return velocity == 0


class ParkSecondTurn(Stage):
    _model = FuzzyModel(
        max_vel=5,
        break_vel=1,
        stop_dist=1.15,
        break_dist=1.7,
        sharpness=0.25,
    )

    def control(self, tank, distances):
        distance = min(distances.se2, distances.sw2)
        velocity = self._model.get_velocity(distance)
        tank.turn_right_circle(velocity)  # Change right to left
        return velocity == 0


class ParkFirstTurn(Stage):
    _model = FuzzyModel(
        max_vel=5,
        break_vel=3,
        stop_dist=6-3.9,
        break_dist=6-2.75,
        sharpness=0.2,
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(6 - distances.ws2)
        tank.turn_left_circle(velocity)  # Change left to right
        return velocity == 0


class DriveCloserSecondTurn(Stage):
    diff = Antecedent(np.linspace(0, 2, 10000), 'diff')
    vel = Consequent(np.linspace(-1, 6, 200), 'vel')

    adj = 0.1

    diff['l'] = fuzz.trapmf(diff.universe, [0, 0, 0.001, 0.01])
    diff['m'] = fuzz.trapmf(diff.universe, [0.001, 0.01, 0.09+adj, 0.1+adj])
    diff['h'] = fuzz.trapmf(diff.universe, [0.09+adj, 0.1+adj, 2, 2])

    vel['z'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['m'] = fuzz.trimf(vel.universe, [1, 2, 3])
    vel['h'] = fuzz.trimf(vel.universe, [4, 5, 6])

    rules = [
        Rule(diff['l'], vel['z']),
        Rule(diff['m'], vel['m']),
        Rule(diff['h'], vel['h']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    @classmethod
    def get_velocity(cls, distances):
        diff = abs(distances.es2 - distances.en2)
        cls.simulation.input['diff'] = diff
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.1:
            return 0
        return velocity

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.turn_right_circle(velocity)  # Change right to left
        stop = velocity == 0
        return stop


class DriveCloserFirstTurn(Stage):
    _model = FuzzyModel(
        max_vel=5,
        break_vel=1,
        stop_dist=0.7,
        break_dist=1,
        sharpness=0.1,
    )

    def control(self, tank, distances):
        distance = distances.ne2
        velocity = self._model.get_velocity(distance)
        tank.turn_left_circle(velocity)  # Change left to right
        stop = velocity == 0
        return stop


class ForwardToExit(Stage):
    _model = FuzzyModel(
        max_vel=10,
        break_vel=3,
        stop_dist=3.9,
        break_dist=4.4,
        sharpness=0.2,
    )

    def control(self, tank, distances):
        distance = 6 - (0 if distances.se2 == 6 else distances.se2)
        velocity = -self._model.get_velocity(distance)  # Negative to move forward
        tank.backward(abs(velocity))  # Use backward movement
        stop = velocity == 0
        
        # If stopped, ensure complete stop
        if stop:
            tank.stop()
        
        return stop