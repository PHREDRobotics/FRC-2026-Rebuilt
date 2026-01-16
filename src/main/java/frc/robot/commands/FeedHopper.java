package frc.robot.commands;

import frc.robot.subsystems.Fuel.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class FeedHopper extends Command {
    
  private HopperSubsystem hopperSubsystem;

  /**
   * Creates a new FeedHopperCommand.
   * 
   * @param subsystem Hopper subsystem
   */
  
  public FeedHopper(HopperSubsystem subsystem) {
    hopperSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    hopperSubsystem.FeedHopper();
  }
  
}