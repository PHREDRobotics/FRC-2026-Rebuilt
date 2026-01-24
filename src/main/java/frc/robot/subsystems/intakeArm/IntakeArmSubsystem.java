package frc.robot.subsystems.intakeArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsystem extends SubsystemBase {

  private SparkMax m_intakeArmMotor;
  private RelativeEncoder m_intakeArmEncoder;
  private double m_encoderValue;

  public IntakeArmSubsystem() {
    m_intakeArmMotor = new SparkMax(Constants.IntakeArmConstants.kIntakeArmMotorCANId, MotorType.kBrushless);
    m_intakeArmEncoder = m_intakeArmMotor.getEncoder();
    m_encoderValue = m_intakeArmEncoder.getPosition();
  }

  public void resetEncoders() {
    m_intakeArmEncoder.setPosition(0);
  }

  public double getIntakeArmEncoder() {
    return m_encoderValue;
  }

  public void startIntakeArmExtend() {
    m_intakeArmMotor.set(Constants.IntakeArmConstants.kIntakeArmExtendPower);
  }

  public void startIntakeArmRetract() {
    m_intakeArmMotor.set(-Constants.IntakeArmConstants.kIntakeArmRetractPower);

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

  @Override
  public void periodic() {
    m_encoderValue = m_intakeArmEncoder.getPosition();
    SmartDashboard.putNumber("intakeArm Power", m_intakeArmMotor.get());
    SmartDashboard.putNumber("intakeArm Encoder", m_encoderValue);
  }
}