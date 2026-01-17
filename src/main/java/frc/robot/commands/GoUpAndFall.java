package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

/**
 * Command for manually climb the robot
 */
public class GoUpAndFall extends Command {

    DoubleSupplier climb_power;
    ClimbSubsystem climb_subsystem;

    /**
     * 
     * @param ClimbPower power for the climb motor
     * @param climbSubsystem
     */
    public GoUpAndFall(DoubleSupplier climbPower, ClimbSubsystem climbSubsystem) {
        this.climb_power = climbPower;
        this.climb_subsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climb_subsystem.resetEncoders();
    }

    @Override
    public void execute() {
        this.climb_subsystem.setRawClimbPower(this.climb_power);
    }

    @Override
    public boolean isFinished() {
        if (this.climb_subsystem.getClimbEncoder() > 100) {
            return true;
        } else {
            return false;
        }
    }
}