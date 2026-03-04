package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Automatically aims and shoots at the hub based on the distance from the hub
 */
public class AutoShootCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private FuelSubsystem m_fuelSubsystem;
  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;

  private double m_x;
  private double m_y;
  private double m_rot;

  private Timer m_timer;

  /**
   * Initialize the auto shoot command
   * @param shooterSubsystem
   * @param fuelSubsystem
   * @param swerveSubsystem
   * @param visionSubsystem
   * @param x The x input of the controller (You can still drive!)
   * @param y The y input of the controller (You can still strafe!)
   */
  public AutoShootCommand(ShooterSubsystem shooterSubsystem, FuelSubsystem fuelSubsystem, SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    m_shooterSubsystem = shooterSubsystem;
    m_fuelSubsystem = fuelSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_x = x.getAsDouble();
    m_y = y.getAsDouble();
    m_rot = rot.getAsDouble();

    m_timer = new Timer();

    addRequirements(shooterSubsystem, fuelSubsystem, swerveSubsystem, visionSubsystem);
  }

  private boolean canShoot() {
    SmartDashboard.putBoolean("Is aligned", m_swerveSubsystem.isAlignedWithHub());
    SmartDashboard.putBoolean("Is at speed", m_shooterSubsystem.isAtSpeed());

    return m_swerveSubsystem.isAlignedWithHub() && m_shooterSubsystem.isAtSpeed() && m_swerveSubsystem.getHubDistance() > Constants.ShooterConstants.kMinimumShootDistanceMeters;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_shooterSubsystem.shoot(m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power ChatGPT", m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power Linear", m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power Root", m_shooterSubsystem.getShootPowerRoot(m_swerveSubsystem.getHubDistance()));
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(2)) {
      m_fuelSubsystem.feed();
      m_shooterSubsystem.shoot(m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    }

    //m_swerveSubsystem.alignToAndDrive(m_x, m_y, new Rotation2d(m_swerveSubsystem.getPointAngleRadians(Constants.VisionConstants.kHubPos)), false);
    m_swerveSubsystem.drive(m_x, m_y, m_rot, true);

    //m_swerveSubsystem.driveRelativeTo(m_visionSubsystem.getEstimatedRelativePose().get(), new Pose2d(m_visionSubsystem.getEstimatedRelativePose().get().getX(), m_visionSubsystem.getEstimatedRelativePose().get().getY(), new Rotation2d(0)));
    if (m_visionSubsystem.hasValidTarget()) {
      m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getLastAverageGlobalPose(), Timer.getFPGATimestamp());
    }

    SmartDashboard.putBoolean("Can Shoot", canShoot());

    SmartDashboard.putBoolean("Is at speed shooter", m_shooterSubsystem.isAtSpeed());

    SmartDashboard.putNumber("shoot power", m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Shoot Power Linear", m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Shoot Power Root", m_shooterSubsystem.getShootPowerRoot(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get point angle degrees", m_swerveSubsystem.getPointAngleDegrees(Constants.VisionConstants.kHubPos));
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_fuelSubsystem.stop();
    m_shooterSubsystem.stop();
  }
}