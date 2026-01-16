package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Configs.IntakeMotor;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {


  public SparkMax intakeMotorSparkMax = new SparkMax(FuelConstants.kHopperMotorCANId, MotorType.kBrushless);


public IntakeSubsystem() {
    intakeMotorSparkMax.configure(Configs.IntakeMotor.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    intakeMotorSparkMax.set(0);
  }


  public void startIntake() {
    intakeMotorSparkMax.set(FuelConstants.kIntakeSpeed);
  }

}
