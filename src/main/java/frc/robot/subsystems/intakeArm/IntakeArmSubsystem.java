package frc.robot.subsystems.intakeArm;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

/**
 * Subsystem for controller the intake arm
 */
public class IntakeArmSubsystem extends SubsystemBase {
  private SparkMax m_intakeArmMotor;
  private RelativeEncoder m_intakeArmEncoder;
  private SparkClosedLoopController m_intakeArmPIDController;
  private double m_encoderValue;
  private double m_manualVolts;

  public IntakeArmSubsystem() {
    m_intakeArmMotor = new SparkMax(Constants.IntakeArmConstants.kIntakeArmMotorCANId, MotorType.kBrushless);
    m_intakeArmEncoder = m_intakeArmMotor.getEncoder();

    m_manualVolts = Constants.IntakeArmConstants.kIntakeArmManualVolts;

    // Initialize the closed loop controller
    m_intakeArmPIDController = m_intakeArmMotor.getClosedLoopController();
    m_intakeArmMotor.configure(Configs.IntakeArmConfig.intakeArmMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  public void resetEncoder() {
    m_intakeArmEncoder.setPosition(0);
  }
  
  public void resetEncoderUp() {
    m_intakeArmEncoder.setPosition(Constants.IntakeArmConstants.kArmHorizontalToUpEncoderValue);
  }

  public void resetEncoderDown() {
    m_intakeArmEncoder.setPosition(Constants.IntakeArmConstants.kArmHorizontalToDownEncoderValue);
  }

  public double getIntakeArmEncoder() {
    return m_encoderValue;
  }

  public void intakeArmLower() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmHorizontalToUpEncoderValue, ControlType.kPosition);
  }

  public void intakeArmMiddle() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmHorizontalToDrivingEncoderValue, ControlType.kPosition);
  }

  public void intakeArmRaise() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmHorizontalToUpEncoderValue, ControlType.kPosition);
  }

  public void setIntakeArmToPosition(double position) {
    m_intakeArmPIDController.setSetpoint(position, ControlType.kPosition);
  }

  public void stopIntakeArm() {
    m_intakeArmMotor.set(0);
  }

  public void setIntakeArm(double volt) {
    m_intakeArmMotor.setVoltage(volt);
  }

  public boolean isIntakeArmExtended() {
    return m_encoderValue <= Constants.IntakeArmConstants.kArmHorizontalToUpEncoderValue;
  }

  public boolean isIntakeArmRetracted() {
    return m_encoderValue >= Constants.IntakeArmConstants.kArmHorizontalToDownEncoderValue;
  }

  public double getVoltageMult() {
    return m_manualVolts;
  }

  public Command lowerIntakeCommand() {
    return Commands.runOnce(() -> this.intakeArmLower(), this);
  }
  
  public Command middleIntakeCommand() {
    return Commands.runOnce(() -> this.intakeArmMiddle(), this);
  }

  public Command raiseIntakeCommand() {
    return Commands.runOnce(() -> this.intakeArmRaise(), this);
  }

  public Command resetArmCommand() {
    return Commands.runOnce(() -> this.resetEncoder(), this);
  }

  public Command resetArmUpCommand() {
    return Commands.runOnce(() -> this.resetEncoderUp(), this);
  }

  public Command resetArmDownCommand() {
    return Commands.runOnce(() -> this.resetEncoderDown(), this);
  }
  // if current ever goes above .625 its joever
  public Command setArmCommand(DoubleSupplier analogInput) {
    return Commands.runEnd(() -> setIntakeArm(analogInput.getAsDouble()*m_manualVolts), () -> stopIntakeArm(), this);
  }

  @Override
  public void periodic() {
    m_encoderValue = m_intakeArmEncoder.getPosition();
    SmartDashboard.setDefaultNumber("Target Position", m_intakeArmPIDController.getMAXMotionSetpointPosition());
    SmartDashboard.setDefaultNumber("Target Velocity", m_intakeArmPIDController.getMAXMotionSetpointVelocity());
    SmartDashboard.putNumber("intakeArm Power", m_intakeArmMotor.get());
    SmartDashboard.putNumber("intakeArm Encoder", m_encoderValue);
    m_manualVolts = SmartDashboard.getNumber("Arm Voltage", Constants.IntakeArmConstants.kIntakeArmManualVolts);
    SmartDashboard.putNumber("Arm Voltage", m_manualVolts);

    // Display encoder position and velocity

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      m_intakeArmEncoder.setPosition(0);
    }

  }

}