package frc.robot.commands;

import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
    
  private IntakeSubsystem IntakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   * 
   * @param subsystem Intake subsystem
   */
  public IntakeUp(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    IntakeSubsystem.moveIntakeUp();
  }
  
}