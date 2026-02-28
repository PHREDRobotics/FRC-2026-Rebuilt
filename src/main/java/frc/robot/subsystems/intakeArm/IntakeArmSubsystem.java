package frc.robot.subsystems.intakeArm;

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

  public IntakeArmSubsystem() {
    m_intakeArmMotor = new SparkMax(Constants.IntakeArmConstants.kIntakeArmMotorCANId, MotorType.kBrushless);
    m_intakeArmEncoder = m_intakeArmMotor.getEncoder();

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

  public double getIntakeArmEncoder() {
    return m_encoderValue;
  }

  public void intakeArmLower() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmDownEncoderValue, ControlType.kPosition);
  }

  public void intakeArmMiddle() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmDrivingEncoderValue, ControlType.kPosition);
  }

  public void intakeArmRaise() {
    m_intakeArmPIDController.setSetpoint(Constants.IntakeArmConstants.kArmUpEncoderValue, ControlType.kPosition);
  }

  public void setIntakeArmToPosition(double position) {
    m_intakeArmPIDController.setSetpoint(position, ControlType.kPosition);
  }

  public void stopIntakeArm() {
    m_intakeArmMotor.set(0);
  }

  public boolean isIntakeArmExtended() {
    return m_encoderValue <= Constants.IntakeArmConstants.kArmUpEncoderValue;
  }

  public boolean isIntakeArmRetracted() {
    return m_encoderValue >= Constants.IntakeArmConstants.kArmDownEncoderValue;
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

  @Override
  public void periodic() {
    m_encoderValue = m_intakeArmEncoder.getPosition();
    SmartDashboard.putNumber("intakeArm Power", m_intakeArmMotor.get());
    SmartDashboard.putNumber("intakeArm Encoder", m_encoderValue);
    // Display encoder position and velocity

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      m_intakeArmEncoder.setPosition(0);
    }

  }

}