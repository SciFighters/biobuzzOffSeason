# Architecture

## Command Modules

- IntakeCmds
  - Intake (forward)
  - IntakeEject (backward)
  - IdleIntake (off)

- LiftCmds
  - LiftGoToHeight (PID to target height in meters)
  - LiftHoldPosition (hold current or specific height)
  - LiftPower (open-loop power for duration)

- BoxCmds
  - Discharge (open servos)
  - Close (close servos)

- DriveCmds
  - TeleopDrive (field-centric mecanum)
  - Rotate (turn to heading)
  - TranslateTo (drive to x,y)
  - ResetPose
  - Wait

## Subsystem Module

- IntakeSub
  - 1 Intake Motor
  - setPower/getPower

- LiftSub
  - 2 Lift Motors
  - setPower/getPower
  - getPos/getHeight/getHeightAvg
  - goToHeight(targetHeightM) - PID to height in meters
  - resetLiftPID()
  - atTargetHeight() / atTargetHeight(toleranceMm)
  - isLevel()

- BoxSub
  - 2 Discharge Servos
  - setServosPosition/getServosPosition

- DriveSub
  - 4 Drive Motors
  - Pinpoint Odometry
  - drive(x, y, rot) - field-centric
  - rotate(targetRad, power, timeout, tolerance)
  - translateTo(targetX, targetY, power, timeout, tolerance)
  - resetHeading()/setPose()
  - getHeadingRad()/getPosX()/getPosY()