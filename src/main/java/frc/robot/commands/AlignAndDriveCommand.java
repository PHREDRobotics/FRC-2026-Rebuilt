package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Aligns to a tag while letting the driver control the x and y component
 */
public class AlignAndDriveCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  double m_x;
  double m_y;

  Translation2d m_position;

  public AlignAndDriveCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, Translation2d position, DoubleSupplier x, DoubleSupplier y) {
    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_x = x.getAsDouble();
    m_y = y.getAsDouble();

    m_position = position;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.alignToAndDrive(m_x, m_y, new Rotation2d(m_swerveSubsystem.getPointAngleRadians(m_position)), false);

    if (m_visionSubsystem.hasValidTarget()) {
      m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    }
  }
}
