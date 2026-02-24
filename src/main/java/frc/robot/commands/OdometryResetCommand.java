package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class OdometryResetCommand extends Command {
    SwerveSubsystem m_swerveSubsystem;
    VisionSubsystem m_visionSubsystem;

    public OdometryResetCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;

        addRequirements(swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (m_visionSubsystem.hasValidTarget()) {
            m_swerveSubsystem.resetOdometry(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
