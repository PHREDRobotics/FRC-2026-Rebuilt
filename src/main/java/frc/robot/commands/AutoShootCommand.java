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

  private DoubleSupplier m_x;
  private DoubleSupplier m_y;
  private DoubleSupplier m_throttle;

  private Timer m_timer;

  /**
   * Initialize the auto shoot command
   * 
   * @param shooterSubsystem
   * @param fuelSubsystem
   * @param swerveSubsystem
   * @param visionSubsystem
   * @param x                The x input of the controller (You can still drive!)
   * @param y                The y input of the controller (You can still strafe!)
   */
  public AutoShootCommand(ShooterSubsystem shooterSubsystem, FuelSubsystem fuelSubsystem,
      SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y,
      DoubleSupplier throttle) {
    m_shooterSubsystem = shooterSubsystem;
    m_fuelSubsystem = fuelSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_x = x;
    m_y = y;
    m_throttle = throttle;

    m_timer = new Timer();

    addRequirements(shooterSubsystem, fuelSubsystem, swerveSubsystem, visionSubsystem);
  }

  private boolean canShoot() {
    SmartDashboard.putBoolean("Is aligned", m_swerveSubsystem.isAlignedWithHub());
    SmartDashboard.putBoolean("Is at speed", m_shooterSubsystem.isAtSpeed());

    return m_swerveSubsystem.isAlignedWithHub() && m_shooterSubsystem.isAtSpeed()
        && m_swerveSubsystem.getHubDistance() > Constants.ShooterConstants.kMinimumShootDistanceMeters;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_shooterSubsystem.shoot(m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power ChatGPT",
        m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power Linear",
        m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Get Shoot Power Root",
        m_shooterSubsystem.getShootPowerRoot(m_swerveSubsystem.getHubDistance()));
    m_timer.start();

  }

  @Override
  public void execute() {
    if (canShoot()) {
      m_fuelSubsystem.feed();
      m_shooterSubsystem.shoot(m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    }

    m_swerveSubsystem.alignToAndDrive(
        m_x.getAsDouble() * m_throttle.getAsDouble(),
        m_y.getAsDouble() * m_throttle.getAsDouble(),
        new Rotation2d(m_swerveSubsystem.getPointAngleRadians(Constants.VisionConstants.getHubPos())),
        false);

    if (m_visionSubsystem.hasValidTarget()) {
      m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    }

    SmartDashboard.putBoolean("Can Shoot", canShoot());

    SmartDashboard.putNumber("shoot power", m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Shoot Power Linear",
        m_shooterSubsystem.getShootPowerLinear(m_swerveSubsystem.getHubDistance()));
    SmartDashboard.putNumber("Shoot Power Root",
        m_shooterSubsystem.getShootPowerRoot(m_swerveSubsystem.getHubDistance()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_fuelSubsystem.stop();
    m_shooterSubsystem.stop();
  }
}