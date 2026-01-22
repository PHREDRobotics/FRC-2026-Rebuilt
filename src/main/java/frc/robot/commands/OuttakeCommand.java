package frc.robot.commands;

import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCommand extends Command {
    
  private IntakeSubsystem IntakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   * 
   * @param subsystem Intake subsystem
   */
  public OuttakeCommand(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    IntakeSubsystem.outtake();
  }

  @Override
  public void execute() {
  }
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stopIntake();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
  
}