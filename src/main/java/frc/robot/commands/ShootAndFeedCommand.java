package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Automatically aims and shoots at the hub based on the distance from the hub
 */
public class ShootAndFeedCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private FuelSubsystem m_fuelSubsystem;
  /**
   * Initialize the feed and shoot command
   * @param shooterSubsystem
   * @param fuelSubsystem
   */
  public ShootAndFeedCommand(ShooterSubsystem shooterSubsystem, FuelSubsystem fuelSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_fuelSubsystem = fuelSubsystem;

    addRequirements(shooterSubsystem, fuelSubsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shoot(Constants.ShooterConstants.kInitialShootingSpeed);
    m_fuelSubsystem.feed();
  }

  @Override
  public void end(boolean interrupted) {
    m_fuelSubsystem.stop();
    m_shooterSubsystem.stop();
  }
}