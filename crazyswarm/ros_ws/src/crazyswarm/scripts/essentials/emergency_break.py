from pycrazyswarm import *

"""Interrupts any high-level command to stop and cut motor power.
Intended for non-emergency scenarios, e.g. landing with the possibility
of taking off again later. Future low- or high-level commands will
restart the motors. Equivalent of :meth:`stop()` when in high-level mode.
"""
def semi_turn_off(swarm):
    for cf in swarm.allcfs.crazyflies:
        cf.cmdStop()

'''cutting power to the motors, it ensures that any future commands, 
both high-level and streaming, will have no effect.'''
def real_turn_off(swarm):
    for cf in swarm.allcfs.crazyflies:
        cf.emergency()