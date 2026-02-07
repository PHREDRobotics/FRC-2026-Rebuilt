package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;

  public ShooterCommand(ShooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shoot(Constants.ShooterConstants.kInitialShootingSpeed);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.shoot(speed.getAsDouble());
    SmartDashboard.putNumber("Target Speed", Constants.ShooterConstants.kInitialShootingSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
