// package frc.robot.subsystems.climber;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;
// import frc.robot.Constants;

// /**
//  * Subsystem for controller the climber
//  */
// public class ClimberSubsystem extends SubsystemBase {
//   private SparkMax m_climberMotor;
//   private RelativeEncoder m_climberEncoder;
//   private SparkClosedLoopController m_climberPID;

//   public ClimberSubsystem() {
//     m_climberMotor = null;//new SparkMax(Constants.ClimberConstants.kClimberMotorCANId, MotorType.kBrushless);
//     //m_climberEncoder = m_climberMotor.getEncoder();
//     //m_climberPID = m_climberMotor.getClosedLoopController();

//     //m_climberMotor.configure(Configs.ClimberConfig.climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//   }

//   public void resetEncoders() {
//     m_climberEncoder.setPosition(0);
//   }

//   public double getClimberEncoderPosition() {
//     return m_climberEncoder.getPosition();
//   }

//   public void setClimberPosition(double position) {
//     m_climberPID.setSetpoint(position, ControlType.kPosition);
//   }

//   public void stopClimber() {
//     m_climberMotor.stopMotor();
//   }

//   public boolean isClimberExtended() {
//     return m_climberEncoder.getPosition() <= Constants.ClimberConstants.kClimberRaisedEncoderValue;
//   }

//   public boolean isRobotClimbed() {
//     return m_climberEncoder.getPosition() <= Constants.ClimberConstants.kClimberClimbedEncoderValue;
//   }

//   public boolean isClimberRetracted() {
//     return m_climberEncoder.getPosition() >= Constants.ClimberConstants.kClimberLoweredEncoderValue;
//   }

//   public Command climbCommand() {
//     return Commands.runOnce(() -> this.setClimberPosition(Constants.ClimberConstants.kClimberClimbedEncoderValue));
//   }

//   public Command extendCommand() {
//     return Commands.runOnce(() -> this.setClimberPosition(Constants.ClimberConstants.kClimberRaisedEncoderValue));
//   }

//   public Command retractCommand() {
//     return Commands.startEnd(() -> this.setClimberPosition(Constants.ClimberConstants.kClimberLoweredEncoderValue), () -> this.stopClimber()).until(() -> isClimberRetracted());
//   }

//   @Override
//   public void periodic() {
//     //SmartDashboard.putNumber("climb Power", m_climberMotor.get());
//     //SmartDashboard.putNumber("climb Encoder", m_climberEncoder.getPosition());


//   }
// }