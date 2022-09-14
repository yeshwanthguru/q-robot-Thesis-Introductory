import qrobot
import json
import numpy as np
from qrobot.models import AngularModel

model = qrobot.models.AngularModel(n=1, tau=30)
sequence = list()

n=1
tau=30
# Balanced events (between 0 and 1)
for i in range(0, int(tau / 2)):
    sequence.append(np.random.randint(0, 1000) / 1000)

# Unbalanced events (balanced between .5 and 1)
for i in range(int(tau / 2), tau):
    sequence.append(np.random.randint(500, 1000) / 1000)


model.clear()  # to re-initialize the model (allows re-runing this cell without double the encoding)

for t in range(0, model.tau):  # loop throug the event sequence
    model.encode(sequence[t], dim=0)

model.print_circuit()


print(f"Hello model!\n{model}")

shots = 1000000
counts = model.measure(shots)



print("Aggregated binary outcomes of the circuit:")
print(json.dumps(counts, sort_keys=True, indent=4))


