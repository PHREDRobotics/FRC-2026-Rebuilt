package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;



/**
 * Constants for the robot
 */
public class Constants {
  public static final class ControllerConstants {
    public static final double kFlightStickXDeadband = 0.2;
    public static final double kFlightStickYDeadband = 0.15;
    public static final double kFlightStickZDeadband = 0.15;

    public static final double kXRateLimit = 20;
    public static final double kYRateLimit = 20;
    public static final double kZRateLimit = 20;

    public static final double kXboxDeadband = 0.15;

    public static final double kThrottleMultiplier = 1;

    public static final double kMaxThrottle = 5;
    public static final double kMinThrottle = 2.5;
  }

  public static final class GyroConstants {
    public static final NavXComType kComType = NavXComType.kMXP_SPI;
  }

  public static final class PhysicalConstants {
    public static final double kRobotMassPounds = 120;

    public static final double kBumperLength = 35.5;
    public static final double kTrackLength = 24;

    public static final double kMaxSpeed = 6;
    public static final double kMaxAngularSpeed = 6;

    public static final double kMaxAcceleration = 3;
    public static final double kMaxAngularAcceleration = 3;

    public static final double kNeoFreeSpeedRpm = 5676;
  }


  public static final class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kDtSeconds = 0.02;

    public static final int kBackLeftDriveMotorCANId = 26;
    public static final int kFrontLeftDriveMotorCANId = 21;
    public static final int kFrontRightDriveMotorCANId = 11;
    public static final int kBackRightDriveMotorCANId = 16;

    public static final int kBackLeftTurnMotorCANId = 27;
    public static final int kFrontLeftTurnMotorCANId = 22;
    public static final int kFrontRightTurnMotorCANId = 12;
    public static final int kBackRightTurnMotorCANId = 17;

    public static final double kDrivingMotorReduction = 8;
    public static final double kTurningMotorReduction = 21;

    public static final Translation2d kFrontLeftLocationInches = new Translation2d(12.25, 12.25);
    public static final Translation2d kFrontRightLocationInches = new Translation2d(12.25, -12.25);
    public static final Translation2d kBackLeftLocationInches = new Translation2d(-12.25, 12.25);
    public static final Translation2d kBackRightLocationInches = new Translation2d(-12.25, -12.25);

    public static final Translation2d kFrontLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kFrontLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kFrontLeftLocationInches.getY()));
    public static final Translation2d kFrontRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kFrontRightLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kFrontRightLocationInches.getY()));
    public static final Translation2d kBackLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kBackLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kBackLeftLocationInches.getY()));
    public static final Translation2d kBackRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kBackRightLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kBackRightLocationInches.getY()));

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        Constants.SwerveConstants.kFrontLeftLocationMeters,
        Constants.SwerveConstants.kFrontRightLocationMeters,
        Constants.SwerveConstants.kBackLeftLocationMeters,
        Constants.SwerveConstants.kBackRightLocationMeters);

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(0.8, 0.8, 0.8);

    public static final double kXYPosP = 0.4;
    public static final double kXYPosI = 0.001;
    public static final double kXYPosD = 0.005;
    public static final TrapezoidProfile.Constraints kXYControllerConstraints = new TrapezoidProfile.Constraints(0.5,
        0.5);

    public static final double kRotP = 0.4;
    public static final double kRotI = 0.001;
    public static final double kRotD = 0.025;
    public static final TrapezoidProfile.Constraints kRotControllerConstraints = new TrapezoidProfile.Constraints(0.25,
        0.5);

    public static final double kXDeadband = 0.03;
    public static final double kYDeadband = 0.03;
    public static final double kRotDeadband = 0.05;
  }

  public static final class VisionConstants {

    public static final double kPositionTolerance = 0.2;// TWEAK
    public static final double kAngleTolerance = 3;// TWEAK

    public static final String kCameraName = "ArducamOV9872 1";


    public static final double k2pi = Math.PI * 2;

    public static final Transform3d kRobotToCamera1 = new Transform3d(0, 0, 0, new Rotation3d());

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Camera facing forward. And offset to the robot center by half a meter up and
    // half a meter forward.
    public static final Transform3d robotToCamera1 = new Transform3d(0.5, 0, 0.5, new Rotation3d());

    public static final double kStrafeMult = 0.5;
    public static final Pose2d kOffset = new Pose2d(1, 0, new Rotation2d(Math.PI));
  }
    public static final class ClimbConstants {
        public static int kClimbControllerPort=1;
    }

}