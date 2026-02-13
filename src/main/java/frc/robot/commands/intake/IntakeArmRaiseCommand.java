package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;

/**
 * Raises the intake arm
 */
public class IntakeArmRaiseCommand extends Command {
  private IntakeArmSubsystem m_armSubsystem;

  public IntakeArmRaiseCommand(IntakeArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    m_armSubsystem.intakeArmRetract();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
