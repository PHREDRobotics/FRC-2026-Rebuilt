package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Command for manually climbing the robot
 */

public class ClimberExtendCommand extends Command {
  private ClimberSubsystem m_climberSubsystem;

  /**
   * 
   * @param climberSubsystem
   */
  public ClimberExtendCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    m_climberSubsystem.setClimberPosition(Constants.ClimberConstants.kClimberRaisedEncoderValue);
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
