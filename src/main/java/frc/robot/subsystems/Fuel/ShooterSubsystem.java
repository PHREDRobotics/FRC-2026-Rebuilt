package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Configs.shooterSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem {


  public SparkMax shooterSparkMax = new SparkMax(FuelConstants.kShooterMotorCANId, MotorType.kBrushless);


public ShooterSubsystem() {
    shooterSparkMax.configure(Configs.shooterSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    shooterSparkMax.set(0);
  }


  public void shoot() {
    shooterSparkMax.set(FuelConstants.kShooterSpeed);
  }

}
