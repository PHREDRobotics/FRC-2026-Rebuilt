package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;

public class ManualArmCommand extends Command {
    IntakeArmSubsystem m_intakeSubsystem;
    DoubleSupplier m_input;

    public ManualArmCommand(IntakeArmSubsystem intakeSubsystem, DoubleSupplier analogInput) {
        m_intakeSubsystem = intakeSubsystem;
        m_input = analogInput;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (SmartDashboard.getNumber("Amps/Arm", Constants.IntakeArmConstants.kArmStallAmps) < Constants.IntakeArmConstants.kArmStallAmps) {
            m_intakeSubsystem.setIntakeArm(m_intakeSubsystem.getVoltageMult() * -m_input.getAsDouble());
        } else {
            m_intakeSubsystem.stopIntakeArm();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntakeArm();
    }

}
