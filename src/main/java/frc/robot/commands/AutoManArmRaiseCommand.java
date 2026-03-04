package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;

public class AutoManArmRaiseCommand extends Command {
    IntakeArmSubsystem m_intakeSubsystem;

    public AutoManArmRaiseCommand(IntakeArmSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.setIntakeArm(Constants.IntakeArmConstants.kIntakeArmManualVolts);
    }

    @Override
    public boolean isFinished(){
        return SmartDashboard.getNumber("Amps/Arm", Constants.IntakeArmConstants.kArmStallAmps) < Constants.IntakeArmConstants.kArmStallAmps;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntakeArm();
    }

}
