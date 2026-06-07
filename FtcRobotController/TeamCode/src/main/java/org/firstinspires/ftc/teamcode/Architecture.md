# Architecture

## Command Modules

- IntakeCmds
  - intake state change (forward/backwards/off)

- LiftCmds
  - height control
  - home
  - manual

- BoxCmds
  - discharge ()

- DriveCmds
  - mechanum drive
  - home to angle
  - GoTo(x,y)

## Subsystem Module

- IntakeSub
  - 1 Intake Motor 
  - Set/Get Power

- LiftSub
  - 2 Lift Motor
  - Set/Get Power
  - Set/Get Position

- BoxSub
  - 2 Discharge Servo
    - Set/Get Position

- DriveSub
  - 4 Drive Motor
