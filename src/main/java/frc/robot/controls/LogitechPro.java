package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

/**
 * Class to adjust controls for the Logitech Pro joystick
 */
public class LogitechPro extends CommandJoystick {
  public LogitechPro(int port) {
    super(port);
  } 

  @Override
  public double getThrottle() { // have to turn 1 -> -1 to 0 -> 1
    return (1 + (-1 * super.getThrottle())) / 2;
  }

  public double getAdjustedThrottle() {
    return (getThrottle() - Constants.ControllerConstants.kMinThrottle) * (Constants.ControllerConstants.kMaxThrottle / (Constants.ControllerConstants.kMaxThrottle - Constants.ControllerConstants.kMinThrottle));
  }

  @Override
  public double getX() {
    double input = super.getX();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickXDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  @Override
  public double getY() {
    double input = super.getY();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickYDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  @Override
  public double getZ() {
    double input = super.getZ();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickZDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }
}
