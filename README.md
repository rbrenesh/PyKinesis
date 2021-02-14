# PyKinesis
Python wrapper for .NET Thorlabs Kinesis

## Usage
You need an active Kinesis installation from Thorlabs. Only TCube motor controllers have been implemented so far. Change:

```
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.TCube.DCServoCLI.dll")
```

to point to wherever the dll's are in your local machine.