package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

/**
 * Class to adjust controls for the Logitech Pro joystick
 */
public class LogitechPro extends CommandJoystick {
  public LogitechPro(int port) {
    super(port);
  } 
  /**
   * Dude is this working or not?
   */
  public double getThrottleLever() { // have to turn 1 -> -1 to 0 -> 1
    return (1 + (-1 * super.getThrottle())) / 2;
  }

  public double getAdjustedThrottle() {
    double throttl = ((getThrottleLever() * (Constants.ControllerConstants.kMaxThrottle - Constants.ControllerConstants.kMinThrottle)) + Constants.ControllerConstants.kMinThrottle);
    //SmartDashboard.putNumber("Joystick/Throttle", throttl);
    return throttl;
  }

  // 0 -> .1
  // 1 -> 1
  // (throttle * (max - min)) + min
  // .5 -> .55
  // 
  @Override
  public double getX() {
    double input = super.getX();
  
    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickXDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  
  public double getCoolerY() {
    double input = super.getY();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickYDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  
  public double getCoolerZ() {
    double input = super.getZ();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickZDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control

    return input;
  }
}
