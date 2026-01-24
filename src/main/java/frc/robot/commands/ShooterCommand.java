package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.google.errorprone.annotations.Var;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.subsystems.fuel.ShooterSubsytem.ShooterSubsystem;

public class ShooterCommand extends Command {
   private ShooterSubsystem shooterSubsystem;
   public ShooterCommand (ShooterSubsystem subsystem) {
    shooterSubsystem = subsystem;
   }

@Override
public void initialize() {
    shooterSubsystem.runShooter(1);
}

@Override
public void execute() {

}

@Override
public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
}

@Override
public boolean isFinished() {
    return false;
}






}
