package frc.robot.subsystems.shooter;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax m_shooterLeftSparkMax = new SparkMax(ShooterConstants.kShooterLeftMotorCANId, MotorType.kBrushless);
  private SparkMax m_shooterRightSparkMax = new SparkMax(ShooterConstants.kShooterRightMotorCANId, MotorType.kBrushless);

  private SparkClosedLoopController m_shooterLeftPID;

  private double shootSpeed;
  private double targetShootSpeed = Constants.ShooterConstants.kInitialShootingSpeed;

  public ShooterSubsystem() {
    // Left is leader, right follows the leader
    m_shooterLeftSparkMax.configure(Configs.ShooterLeftConfig.shooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_shooterRightSparkMax.configure(Configs.ShooterRightConfig.shooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_shooterLeftPID = m_shooterLeftSparkMax.getClosedLoopController();

    SmartDashboard.putNumber("Shooter Speed Adjuster", targetShootSpeed);
  }

  public void shoot(double shootSpeedRPM) {
    this.shootSpeed = shootSpeedRPM;

    m_shooterLeftPID.setSetpoint(shootSpeedRPM, ControlType.kVelocity);
  }

  public void stop() {
    m_shooterLeftSparkMax.stopMotor();
  }

  public boolean isAtSpeed() {
    return (m_shooterLeftSparkMax.getEncoder().getVelocity() > shootSpeed - Constants.ShooterConstants.kShootThreshold) && (m_shooterLeftSparkMax.getEncoder().getVelocity() < shootSpeed + Constants.ShooterConstants.kShootThreshold); 
  }

  /**
   * Gets the power for the shooter to shoot at the hub based on a distance in meters
   * @return
   */
  public double getShootPower(double distance) {
    return Constants.ShooterConstants.kAutoShooterDistanceMultiplier * Math.pow(Constants.ShooterConstants.kAutoShooterDistanceExponent, distance);
  }

  public Command shootCommand(DoubleSupplier speed) {
    return Commands.startEnd(() -> this.shoot(speed.getAsDouble()), () -> this.stop());
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("sparkmax encoder", m_shooterLeftSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("velocity", m_shooterLeftSparkMax.getEncoder().getVelocity());
    targetShootSpeed = SmartDashboard.getNumber("Shooter Speed Adjuster", Constants.ShooterConstants.kInitialShootingSpeed);
    SmartDashboard.putNumber("Shooter Speed Adjuster", targetShootSpeed);
  }
}
