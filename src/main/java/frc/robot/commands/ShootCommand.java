package frc.robot.commands;

import frc.robot.subsystems.Fuel.ShooterSubsystem;
import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;

public class ShootCommand extends Command {
    private ShooterSubsystem ShooterSubsystem;

  /**
   * Creates a new ShootCommand.
   * 
   * @param subsystem Shooter subsystem
   */
  public ShootCommand(ShooterSubsystem subsystem) {
    ShooterSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    ShooterSubsystem.shoot();
  }
  
}