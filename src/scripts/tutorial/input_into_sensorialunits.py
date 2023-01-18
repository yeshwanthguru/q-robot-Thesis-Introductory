import qrobot
import json
import numpy as np
from qrobot.models import AngularModel
from qrobot.qunits import SensorialUnit

#  AngularModel class with n=1
model = qrobot.models.AngularModel(n=1, tau= 1)

# the SensorialUnit class
sensorial_unit = SensorialUnit("sensorial_unit", Ts=1)

# Generate a sequence of balanced and unbalanced events
sequence = list()
for i in range(0, int(30/2)):
    sequence.append(np.random.randint(0, 1000) / 1000)
for i in range(int(30/2), 30):
    sequence.append(np.random.randint(500, 1000) / 1000)

# Encode the event sequence into the quantum state
for t in range(0, 30):
    model.encode(sequence[t], dim=0)

# Measure the quantum state
shots = 1000000
counts = model.measure(shots)

# Print the aggregated binary outcomes of the circuit
print("Aggregated binary outcomes of the circuit:")
print(json.dumps(counts, sort_keys=True, indent=4))
print(model.circ)
