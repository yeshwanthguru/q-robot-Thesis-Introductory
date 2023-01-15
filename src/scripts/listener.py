import json
import time
import random

from qrobot.models import AngularModel

# Initialize a AngularModel with single qubit:
n = 1  # single qubit
tau = 1  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)

t_index = 1
switch_signal = 0

while True:
    # A switch ("ON/OFF" signal) is read by the python script as input
    switch_signal = int(input("Enter 1 for On and 0 for Off: "))
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
        t_index = 1
        model.clear()
