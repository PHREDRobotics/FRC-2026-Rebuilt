package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Subsystem for controller all vision aspects of the robot
 */
public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera m_camera;

  private Transform3d m_robotToTarget;

  private PhotonPipelineResult result = new PhotonPipelineResult();

  private LinearFilter m_xMeasurementFilter = LinearFilter.movingAverage(20);
  private LinearFilter m_yMeasurementFilter = LinearFilter.movingAverage(20);
  private LinearFilter m_rotMeasurementFilter = LinearFilter.movingAverage(20);

  private double m_xFilterValue = 0;
  private double m_yFilterValue = 0;
  private double m_rotFilterValue = 0;

  private LinearFilter m_xRelativeMeasurementFilter= LinearFilter.singlePoleIIR(0.4, 0.02);
  private LinearFilter m_yRelativeMeasurementFilter = LinearFilter.singlePoleIIR(0.4, 0.02);
  private LinearFilter m_rotRelativeMeasurementFilter = LinearFilter.singlePoleIIR(0.4, 0.02);

  private double m_xRelativeFilterValue = 0;
  private double m_yRelativeFilterValue = 0;
  private double m_rotRelativeFilterValue = 0;

  private PhotonPoseEstimator m_photonPoseEstimator;

  public VisionSubsystem() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);

    m_photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, VisionConstants.kRobotToCamera1);
  }

  /**
   * Returns true if a valid april tag is seen
   * @return
   */
  public boolean hasValidTarget() {
    if (result.hasTargets()) {
      return true;
    } else {
      return false;
    }
  }


  /**
   * Gets the estimated pose of the robot relative to the field
   * 
   * @return The estimated robot pose
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    return visionEst;
  }

  public Pose2d getLastAverageGlobalPose() {
    return new Pose2d(m_xFilterValue, m_yFilterValue, new Rotation2d(m_rotFilterValue));
  }

  public Pose2d getLastAverageRelativePose() {
    return new Pose2d(m_xRelativeFilterValue, m_yRelativeFilterValue, new Rotation2d(m_rotRelativeFilterValue));
  }

  public Optional<Pose2d> getEstimatedRelativePose() {
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Transform3d tagToCamera = cameraToTag.inverse();
    Pose3d cameraPose = new Pose3d(tagToCamera.getX(), tagToCamera.getY(), tagToCamera.getZ(),
        tagToCamera.getRotation());

    Pose3d robotPose = cameraPose.transformBy(VisionConstants.kRobotToCamera1.inverse());
    SmartDashboard.putString("currentRelativePose", robotPose.toString());
    return Optional.ofNullable(robotPose.toPose2d());
  }

  @Override
  public void periodic() {
    var results = m_camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      result = results.get(results.size() - 1);
      SmartDashboard.putBoolean("Has valid target?", result.hasTargets());
      //SmartDashboard.putBoolean("Estimated pose/hasTargets", result.hasTargets());
      if (result.hasTargets()) {
        /*for (int i = 0; i < results.size(); i++) {
          if (results.get(i).getBestTarget().getPoseAmbiguity() > 0.5) {
            results.remove(i);
          }
        } */
        m_robotToTarget = VisionConstants.kRobotToCamera1.plus(result.getBestTarget().getBestCameraToTarget());

        m_xFilterValue = m_xMeasurementFilter.calculate(getEstimatedGlobalPose().get().estimatedPose.getX());
        m_yFilterValue = m_yMeasurementFilter.calculate(getEstimatedGlobalPose().get().estimatedPose.getY());
        m_rotFilterValue = m_rotMeasurementFilter.calculate(getEstimatedGlobalPose().get().estimatedPose.getRotation().toRotation2d().getRadians());

        m_xRelativeFilterValue = m_xRelativeMeasurementFilter.calculate(getEstimatedRelativePose().get().getX());
        m_yRelativeFilterValue = m_yRelativeMeasurementFilter.calculate(getEstimatedRelativePose().get().getY());
        m_rotRelativeFilterValue = m_rotRelativeMeasurementFilter.calculate(getEstimatedRelativePose().get().getRotation().getRadians());

        SmartDashboard.putNumber("robotToTarget/X", m_robotToTarget.getX());
        SmartDashboard.putNumber("robotToTarget/Y", m_robotToTarget.getY());
        SmartDashboard.putNumber("robotToTarget/Z", m_robotToTarget.getZ());
        SmartDashboard.putNumber("robotToTarget/Rot", m_robotToTarget.getRotation().toRotation2d().getRadians());

        SmartDashboard.putNumber("Estimated pose/X", getEstimatedGlobalPose().get().estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("Estimated pose/Y", getEstimatedGlobalPose().get().estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("Estimated pose/Z", getEstimatedGlobalPose().get().estimatedPose.toPose2d().getRotation().getDegrees());
      } else {
        m_xFilterValue = m_xMeasurementFilter.calculate(0);
        m_yFilterValue = m_yMeasurementFilter.calculate(0);
        m_rotFilterValue = m_rotMeasurementFilter.calculate(0);

        m_xRelativeFilterValue = m_xRelativeMeasurementFilter.calculate(0);
        m_yRelativeFilterValue = m_yRelativeMeasurementFilter.calculate(0);
        m_rotRelativeFilterValue = m_rotRelativeMeasurementFilter.calculate(0);
      }
    } else {
      SmartDashboard.putBoolean("Has valid target?", result.hasTargets());
    }
  }
}