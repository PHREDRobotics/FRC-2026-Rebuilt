package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Follows a tag
 * (not field relative)
 */
public class FollowTagCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;

  public FollowTagCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    addRequirements(swerveSubsystem);
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    // m_swerveSubsystem.resetPIDs(offset);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.driveRelativeTo(m_visionSubsystem.getLastAverageRelativePose(), VisionConstants.kOffset);
  }
}