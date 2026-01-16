package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HopperSubsystem {


  public SparkMax hopperMotorSparkMax = new SparkMax(FuelConstants.kHopperMotorCANId, MotorType.kBrushless);


public HopperSubsystem() {
    hopperMotorSparkMax.configure(Configs.HopperMotor.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

    public void stop() {
    hopperMotorSparkMax.set(0);
  }


  public void FeedHopper() {
    hopperMotorSparkMax.set(FuelConstants.kHopperIntakeSpeed);
  }

}
