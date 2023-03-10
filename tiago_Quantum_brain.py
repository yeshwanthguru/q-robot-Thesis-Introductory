import qrobot
import time
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst,OneBurst
import json
from random import randint
from IPython.display import clear_output

Ts = 0.5
Sensorial_unit0 = qrobot.qunits.SensorialUnit("Sensorial_unit0", Ts=0.1)
Sensorial_unit1 = qrobot.qunits.SensorialUnit("Sensorial_unit1", Ts=0.1)
Sensorial_unit2 = qrobot.qunits.SensorialUnit("Sensorial_unit2", Ts=0.1)

# Create a new QUnit that receives inputs from all three sensorial units
Qunit = qrobot.qunits.QUnit(
    name="Qunit",
    model=AngularModel(n=3, tau=25),
    burst=ZeroBurst(),
    Ts=0.2,
    in_qunits={
        0: Sensorial_unit0.id,
        1: Sensorial_unit1.id,
        2: Sensorial_unit2.id,
    },
)


Qunit.query = [0.8,0.5,0.6]


Sensorial_unit0.start()
Sensorial_unit1.start()
Sensorial_unit2.start()
Qunit.start()

statuses = []
refresh_time = 0.5  # Read statuses every 0.5 seconds

for i in range(int(10 * (1 / refresh_time))):
    # Wait and then clean the output
    time.sleep(refresh_time)
    clear_output(wait=True)

    # Change input every 2 second
    if (i * refresh_time) % 2 == 0:
        Sensorial_unit0.scalar_reading = randint(0, 500) / 1000
        Sensorial_unit1.scalar_reading = randint(0, 500) / 1000
        Sensorial_unit2.scalar_reading = randint(0, 500) / 1000

    # Read statused and store it
    status = qrobot.qunits.redis_utils.redis_status()
    statuses.append(status)

    # Extract output values for all QUnits
    for key, value in status.items():
        if ' class' in key and value == 'QUnit':
            qunit_name = key[:-6]  # Remove ' output' suffix
            qunit_output = status.get(f'{qunit_name} output')
            if qunit_output is not None:
                qunit_output = float(qunit_output)  # Convert to float
                if qunit_output > 0.5:
                    print("hi")
                else:
                    print("bye")
                print(f'{qunit_name} output:', qunit_output)
                
    # Print output
    print(json.dumps(status, indent=1, sort_keys=True))
    print(int(i * refresh_time), "/30 seconds")


    # Plot graph
    qbrain_graph = qrobot.graph(status)
    qrobot.draw(qbrain_graph)

Sensorial_unit0.stop()
Sensorial_unit1.stop()
Sensorial_unit2.stop()
Qunit.stop()
