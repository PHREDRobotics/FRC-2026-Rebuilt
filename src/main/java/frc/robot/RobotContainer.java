// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shoot.AutoShootCommand;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final FuelSubsystem m_fuelSubsystem;

  
  private final AutoFactory autoFactory;

  LogitechPro joystick;
  XboxController gamepad;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_fuelSubsystem = new FuelSubsystem();

    autoFactory = new AutoFactory(
        m_swerveSubsystem::getPose,
        m_swerveSubsystem::resetOdometry,
        m_swerveSubsystem::followTrajectory,
        true,
        m_swerveSubsystem);

    joystick = new LogitechPro(0);
    // CommandJoystick buttonBox = new CommandJoystick(1);

    configureBindings();
    Command testTrajectory = autoFactory.trajectoryCmd("TestPath");
    gamepad = new XboxController(1);
  
    configureBindings(); 
  }

  private void configureBindings() {
    // -- Triggers --

    Trigger shooterButton = new Trigger(joystick.button(1));

    // -- Button Assignments --

    shooterButton.whileTrue(new AutoShootCommand(m_shooterSubsystem, m_fuelSubsystem, m_swerveSubsystem, m_visionSubsystem, joystick::getX, joystick::getY));

    // -- Default commands --

    m_swerveSubsystem.setDefaultCommand(new DriveCommand(m_swerveSubsystem,
      joystick::getY,
      joystick::getX, 
      joystick::getZ,
      joystick::getAdjustedThrottle,
      joystick.button(1)));
  }

  public Command testAuto() {
    return Commands.sequence(
        autoFactory.resetOdometry("TestPath"),
        autoFactory.trajectoryCmd("TestPath"));
  }


  public Command getAutonomousCommand() {
    return testAuto();
  }
}
