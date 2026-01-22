package frc.robot.subsystems.fuel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs;

public class ShooterSubsytem extends SubsystemBase {

    SparkMax motor1;

    public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor =
        new SparkMax(10, MotorType.kBrushless);

    public ShooterSubsystem() {
        shooterMotor.configure(Configs.shooterSparkMax.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runShooter(double speed) {
        shooterMotor.set(speed);
    }

    public void stopShooter() {
        shooterMotor.stopMotor();
    }
}

    // public ShooterSubsytem() {
    //     motor1 = new SparkMax(0, MotorType.kBrushless);
    // }

    // public void shoot() {
    //     motor1.set(1);
    // }

    // public void stop() {
    //     motor1.set(0);
    // }
}
/*
 * four motors
 * 
 * shoot when we press a button
 * 
 * stop when we stop pressing the button
 */
