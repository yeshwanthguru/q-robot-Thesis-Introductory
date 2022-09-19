import qrobot
import json
import numpy as np
from qrobot.models import AngularModel

n=1  #switch data with an AngularModel with n=1
tau=15#temporal window

sequence = list()

# Balanced events (between 0 and 1)
for i in range(0, int(tau / 2)):
    sequence.append(np.random.randint(0, 1000) / 1000)

# Unbalanced events (balanced between .5 and 1)
for i in range(int(tau / 2), tau):
    sequence.append(np.random.randint(500, 1000) / 1000)

model = qrobot.models.AngularModel(n , tau) #Angular model is initialized

print(f"Hello model!\n{model}")

#Start a loop: every sample period
for t in range(model.tau): 
     while True:
        model = AngularModel(n, tau)#This input is then encoded in the AngularModel
        model.clear()
        for t in range(0, model.tau): 
            model.encode(sequence[t], dim=0)
            model.print_circuit()
            model.clear()#If we are at the end of the temporal window:
            shots = 1  #Measure the module (Single shot)
            counts = model.measure(shots)
            print("Aggregated binary outcomes of the circuit:")
            print(json.dumps(counts, sort_keys=True, indent=4))
            #Reinitializing the model to encode a new temporal window

            model.clear()

