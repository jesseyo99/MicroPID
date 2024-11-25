# MicroPID
This is a simple PID library that is usuable for any Python projects especially embedded work.

# Usage
Just copy the file into your working folder or internal file system in you microcontroller and import the library.
## Examples
Zero setpoint and no output limits
```python
from microPID import PID
import time
dt = 1

controller = PID(kp = 1, ki = 1, kd = 1)

while 1:
    inputVar = system.getOutput()
    
    output = controller.compute(inputVar,dt)
    
    system.doSomething(output)

    time.sleep(dt)


```

Custom setpoint and output limits
```python
from microPID import PID
import time
dt = 1

controller = PID(kp = 1, ki = 1, kd = 1,setpoint = 100, output_limits=(-100,100))

while 1:
    inputVar = system.getOutput()
    
    output = controller.compute(inputVar,dt)
    
    system.doSomething(output)

    time.sleep(dt)
```
