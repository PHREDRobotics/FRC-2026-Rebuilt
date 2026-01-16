package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private SparkMax climbMotor;
    private RelativeEncoder climbEncoder;
    private AHRS navXMicro;

    /**
     * Subsystem to control the climb
     */
    public ClimbSubsystem() {
        climbMotor = new SparkMax(Constants.ClimbConstants.kClimbControllerPort, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();
        try {
            navXMicro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);

        addChild("NavX Micro", navXMicro);
    }

    /**
     * Reset climb Motor Encoders
     */
    public void resetEncoders() {
        climbMotor.set(0);

        climbEncoder.setPosition(0);
    }

    /**
     * Automatically make climbs the robot, must be called in a loop
     * 
     * @param speed Value between 0 and 1
     */
    public void AutoClimb(double speed) {
        double roll = navXMicro.getRoll();
        boolean checkLevel = false;
        climbMotor.setIdleMode(IdleMode.kBrake);
        if (roll > 1) {
            climbMotor.set(0);
            checkLevel = true;
        } 
        // else if (roll < -1) {
            // rightLiftMotor.set(0);
            // checkLevel = true;
        // } 
        else if (checkLevel && roll < 1 && roll > -1) {
            climbMotor.set(0);
        }
    }

    public double getLeftEncoder() {
        return climbEncoder.getPosition();
    }

    public void extendLeftLift() {
        if (climbEncoder.getPosition() == 0) {

            climbMotor.set(0);

        } else {

            climbMotor.set(.1);

        }
    }



    /**
     * Set the power of the climb motor
     * 
     * @param left_power Value between 0 and 1
     */
    public void setRawLeftPower(DoubleSupplier left_power) {
        climbMotor.set(left_power.getAsDouble());
    }



    public void changeClimbMode(IdleMode mode) {
        climbMotor.setIdleMode(mode);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Power", climbMotor.get());
        SmartDashboard.putNumber("Left Encoder", climbEncoder.getPosition());
        SmartDashboard.putNumber("NavX y", navXMicro.getPitch());
        SmartDashboard.putNumber("NavX x", navXMicro.getRoll());
        SmartDashboard.putNumber("NavX z", navXMicro.getYaw());

    }
}