package frc.robot.commands.fuel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.fuel.FuelSubsystem;

/**
 * Command for outtaking fuel
 */
public class OuttakeCommand extends Command {
    private FuelSubsystem m_fuelSubsystem;

    public OuttakeCommand(FuelSubsystem fuelSubsystem) {
        m_fuelSubsystem = fuelSubsystem;

        addRequirements(m_fuelSubsystem);
    }

    @Override
    public void initialize() {
        m_fuelSubsystem.outtake();
    }
}
