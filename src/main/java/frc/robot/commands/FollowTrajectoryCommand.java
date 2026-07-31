package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowTrajectoryCommand extends Command {
    SwerveSubsystem m_swerveSubsystem;
    VisionSubsystem m_visionSubsystem;

    int split;

    Optional<Trajectory<SwerveSample>> m_trajectory;
    Timer m_timer = new Timer();

    public FollowTrajectoryCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, String trajectory) {
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;

        this.split = 0;

        m_trajectory = Choreo.loadTrajectory(trajectory);

        m_timer.reset();
        m_timer.start();

        addRequirements(swerveSubsystem, visionSubsystem);
    }

    public FollowTrajectoryCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, String trajectory, int split) {
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;

        this.split = split;

        m_trajectory = Choreo.loadTrajectory(trajectory);

        m_timer.reset();
        m_timer.start();

        addRequirements(swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (m_trajectory.isPresent() && DriverStation.getAlliance().isPresent()) {
            m_trajectory = m_trajectory.get().getSplit(split);

            Optional<Pose2d> initialPose = m_trajectory.get().getInitialPose(DriverStation.getAlliance().get() == Alliance.Red);
        
            if (initialPose.isPresent()) {
                m_swerveSubsystem.resetOdometry(initialPose.get());
            }
        }
    }

    @Override
    public void execute() {
        if (m_trajectory.isPresent() && DriverStation.getAlliance().isPresent()) {
            Optional<SwerveSample> sample = m_trajectory.get().sampleAt(m_timer.get(), DriverStation.getAlliance().get() == Alliance.Red);
        
            if (sample.isPresent()) {
                m_swerveSubsystem.followTrajectory(sample.get());
            }
        }

        if (m_visionSubsystem.hasValidTarget()) {
            m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        }
    }
}
