package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Command to bring the climber all the way down so we can go under the trench
 */
public class ClimberRetractCommand extends Command {
  private ClimberSubsystem m_climberSubsystem;

  /**
   * 
   * @param climberSubsystem
   */
  public ClimberRetractCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    m_climberSubsystem.setClimberPosition(Constants.ClimberConstants.kClimberLoweredEncoderValue);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return m_climberSubsystem.isClimberExtended();
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopClimber();
  }
}
