package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Class to adjust controls for the Logitech Pro joystick
 */
public class Xbox extends CommandXboxController {
  SlewRateLimiter lx_limiter = new SlewRateLimiter(1.5);
  SlewRateLimiter ly_limiter = new SlewRateLimiter(1.5);
  SlewRateLimiter rx_limiter = new SlewRateLimiter(1.5);
  SlewRateLimiter ry_limiter = new SlewRateLimiter(1.5);

  public Xbox(int port) {
    super(port);
  } 

  @Override
  public double getLeftX() {
    double input = super.getLeftX();
  
    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kDeadband); // Deadband
    //input = lx_limiter.calculate(input);
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  @Override
  public double getLeftY() {
    double input = super.getLeftY();
  
    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kDeadband); // Deadband
    //input = ly_limiter.calculate(input);
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  @Override
  public double getRightX() {
    double input = super.getRightX();
  
    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kDeadband); // Deadband
    //input = rx_limiter.calculate(input);
    input = input * Math.abs(input); // Square for better control

    return input;
  }

  @Override
  public double getRightY() {
    double input = super.getRightY();
  
    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kDeadband); // Deadband
    //input = ry_limiter.calculate(input);
    input = input * Math.abs(input); // Square for better control

    return input;
  }
}
