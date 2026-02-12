package frc.robot.commands.fuel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.fuel.FuelSubsystem;

public class IntakeCommand extends Command {
    private FuelSubsystem m_fuelSubsystem;

    public IntakeCommand(FuelSubsystem fuelSubsystem) {
        m_fuelSubsystem = fuelSubsystem;

        addRequirements(m_fuelSubsystem);
    }

    @Override
    public void initialize() {
        m_fuelSubsystem.intake();
    }
}
