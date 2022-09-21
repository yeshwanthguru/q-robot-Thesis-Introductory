"""
PSEUDOCODE:
- Initialize a AngularModel with single qubit
- Start a loop: every sample period:
  - A switch ("ON/OFF" signal) is read by the python script as input
  - This input is then encoded in the AngularModel
  - If we are at the end of the temporal window:
    - Measure the module (single shot, we do not need statistics)
    - Output the result with print or in a log file
    - Reinitialize the model to encode a new temporal window
"""
import json
import time
import random

from qrobot.models import AngularModel

Ts = 0.5  # sample every 500ms

# Initialize a AngularModel with single qubit:
n = 1  # single qubit
tau = 10  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)

# Start a loop
t_index = 1
while True:
    # Every sample period:
    time.sleep(Ts)
    # A switch ("ON/OFF" signal) is read by the python script as input
    switch_signal = random.randint(0, 1)
    print("switch_signal =", switch_signal)
    # This input is then encoded in the AngularModel
    model.encode(switch_signal, dim=0)
    # If we are not at the end of the temporal window:
    if t_index < tau:
        t_index += 1
    # If we are at the end of the temporal window:
    else:
        # Print the final circuit
        print(model.circ)
        # Measure the module
        result = model.measure()
        # Output the result with print or in a log file
        print("result =", result)
        # Reinitialize the model to encode a new temporal window
        t_index = 0
        model.clear()
