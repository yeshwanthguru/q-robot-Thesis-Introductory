import qrobot
import json
import numpy as np
from qrobot.models import AngularModel
from contextlib import redirect_stdout

n=1 #switch data with an AngularModel with n=1
tau=15 #temporal window

#initialized an angular model
model = qrobot.models.AngularModel(n,tau)
sequence = list()  

#start a loop for every sample period
for time in range(0, model.tau):  # loop throug the event sequence
 
#A virtual switch yet to be done in the final script

# Balanced events between the switch_on and switch_off(between 0 and 1)
 for i in range(1, int(tau / 2)):
    sequence.append(np.random.randint(0, 1000) / 1000)

# Unbalanced events (balanced between .5 and 1)
for i in range(int(tau / 2), tau):
    sequence.append(np.random.randint(500, 1000) / 1000)

 

#Encode the input in the model
model.encode(sequence[time], dim=0)


model.print_circuit()

#Measure the module (Single shot)
shots = 1
counts = model.measure(shots)


#output
with open('introductory_demo_out.txt', 'w') as f:
     with redirect_stdout(f):
          print("Aggregated binary outcomes of the circuit:")
          print(json.dumps(counts, sort_keys=True, indent=4))

#Reinitializing the model to encode a new temporal window

model.clear()

