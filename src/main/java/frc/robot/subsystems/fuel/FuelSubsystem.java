package frc.robot.subsystems.fuel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

/**
 * Subsystem for controlling all elements that move fuel around except for the shooter
 * (Intake motors, hopper motors, vector motors, and feeder motors)
 */
public class FuelSubsystem extends SubsystemBase {
  enum FuelState {
    Intaking,
    Outtaking,
    Feeding,
    Stop 
  }

  private SparkMax m_intakeMotor = new SparkMax(Constants.FuelConstants.kIntakeMotorCANId, MotorType.kBrushless);
  private SparkMax m_hopperMotor = new SparkMax(Constants.FuelConstants.kHopperMotorCANId, MotorType.kBrushless);
  private SparkMax m_vectorMotor = new SparkMax(Constants.FuelConstants.kVectorMotorCANId, MotorType.kBrushless);
  private SparkMax m_feederLeftMotor = new SparkMax(Constants.FuelConstants.kFeederLeftMotorCANId, MotorType.kBrushless);
  private SparkMax m_feederRightMotor = new SparkMax(Constants.FuelConstants.kFeederRightMotorCANId,MotorType.kBrushless);

  private FuelState m_fuelState = FuelState.Stop;

  public FuelSubsystem() {
    m_feederLeftMotor.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_feederRightMotor.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeMotor.configure(Configs.FeederConfig.intakeMotorConfig,  ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Starts intake, hopper, and vector motors
   */
  public void intake() {
    m_intakeMotor.set(Constants.FuelConstants.kIntakeSpeed);
    // m_hopperMotor.set(Constants.FuelConstants.kHopperSpeed);
    // m_vectorMotor.set(Constants.FuelConstants.kVectorSpeed);
    // m_feederLeftMotor.set(Constants.FuelConstants.kFeederSpeed);
    // m_feederRightMotor.set(Constants.FuelConstants.kFeederSpeed);

    m_fuelState = FuelState.Intaking;
  }

  /**
   * Starts intake, hopper, and vector motors in reverse to spit out and stuck fuel
   */
  public void outtake() {
    m_intakeMotor.set(-Constants.FuelConstants.kIntakeSpeed);
    m_hopperMotor.set(-Constants.FuelConstants.kHopperSpeed);
    m_vectorMotor.set(-Constants.FuelConstants.kVectorSpeed);
    m_feederLeftMotor.set(-Constants.FuelConstants.kFeederSpeed);

    m_fuelState = FuelState.Outtaking;
  }

  /**
   * Starts hopper, vector and feeder motors
   */
  public void feed() {
    m_hopperMotor.set(Constants.FuelConstants.kHopperSpeed);
    m_vectorMotor.set(Constants.FuelConstants.kVectorSpeed);
    m_feederLeftMotor.set(Constants.FuelConstants.kFeederSpeed);
    m_feederRightMotor.set(Constants.FuelConstants.kFeederSpeed);

    m_fuelState = FuelState.Feeding;
  }

  public void stop() {
    m_intakeMotor.stopMotor();
    m_hopperMotor.stopMotor();    
    m_vectorMotor.stopMotor();
    m_feederLeftMotor.stopMotor();
    m_feederRightMotor.stopMotor();

    m_fuelState = FuelState.Stop;
  }

  public Command intakeCommand() {
    return Commands.startEnd(() -> this.intake(), () -> this.stop());
  }

  public Command outtakeCommand() {
    return Commands.startEnd(() -> this.outtake(), () -> this.stop());
  }

  public Command feedCommand() {
    return Commands.startEnd(() -> this.feed(), () -> this.stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Fuel State", m_fuelState.toString());


  }

}