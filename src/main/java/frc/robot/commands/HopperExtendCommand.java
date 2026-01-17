package frc.robot.commands;

import frc.robot.subsystems.Fuel.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperExtendCommand extends Command {
    private HopperSubsystem HopperSubsystem;

  /**
   * Creates a new HopperExtendCommand.
   * 
   * @param subsystem Hopper subsystem
   */
  public HopperExtendCommand(HopperSubsystem subsystem) {
    HopperSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    HopperSubsystem.extendHopper();
  }
  
}