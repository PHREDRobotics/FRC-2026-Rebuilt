// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climb.ClimberClimbCommand;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.shoot.AutoShootCommand;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Class to connect everything in the robot together
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final FuelSubsystem m_fuelSubsystem;
  private final IntakeArmSubsystem m_intakeArmSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  
  private final AutoFactory autoFactory;

  LogitechPro joystick;
  CommandXboxController gamepad;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_fuelSubsystem = new FuelSubsystem();
    m_intakeArmSubsystem = new IntakeArmSubsystem();
    m_climberSubsystem = new ClimberSubsystem();

    autoFactory = new AutoFactory(
        m_swerveSubsystem::getPose,
        m_swerveSubsystem::resetOdometry,
        m_swerveSubsystem::followTrajectory,
        true,
        m_swerveSubsystem);

    joystick = new LogitechPro(0);
    gamepad = new CommandXboxController(1);

    configureBindings(); 
  }

  private void configureBindings() {
    // -- Triggers --

    Trigger shooterButton = new Trigger(gamepad.y());
    Trigger manShootButton = new Trigger(gamepad.b());

    Trigger intakeButton = new Trigger(gamepad.a());

    Trigger armUpButton = new Trigger(gamepad.start());
    Trigger armDownButton = new Trigger(gamepad.back());

    Trigger climberClimbButton = new Trigger(gamepad.povLeft());
    Trigger climberExtendButton = new Trigger(gamepad.povUp());
    Trigger climberRetractButton = new Trigger(gamepad.povDown());
  
    // -- Button Assignments --

    shooterButton.whileTrue(new AutoShootCommand(m_shooterSubsystem, m_fuelSubsystem, m_swerveSubsystem, m_visionSubsystem, joystick::getX, joystick::getY));
    manShootButton.whileTrue(m_shooterSubsystem.shootCommand(() -> Constants.ShooterConstants.kInitialShootingSpeed));

    intakeButton.toggleOnTrue(m_fuelSubsystem.intakeCommand());

    armUpButton.onTrue(m_intakeArmSubsystem.raiseIntakeCommand());
    armDownButton.onTrue(m_intakeArmSubsystem.lowerIntakeCommand());

    climberClimbButton.onTrue(new ClimberClimbCommand(m_climberSubsystem));
    climberExtendButton.onTrue(new ClimberExtendCommand(m_climberSubsystem));
    climberRetractButton.onTrue(new ClimberRetractCommand(m_climberSubsystem));

    // -- Default commands --

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      joystick::getY,
      joystick::getX, 
      joystick::getZ,
      joystick::getAdjustedThrottle,
      joystick.button(1)));
  }

  // Autos
  
  /**
   * Auto for testing purposes
   * @return
   */
  public Command testAuto() {
    return Commands.sequence(
        autoFactory.resetOdometry("TestPath"),
        autoFactory.trajectoryCmd("TestPath"));
  }

  /**
   * Shoots at the hub
   * @return
   */
  public Command shootHub() {
    return new WaitCommand(2).raceWith(new AutoShootCommand(m_shooterSubsystem, m_fuelSubsystem, m_swerveSubsystem, m_visionSubsystem, () -> 0, () -> 0));
  }

  /**
   * Shoots then climbs starting from the left position relative to the drivers
   * @return
   */
  public Command ShootClimbPositionLeft() {
    return Commands.sequence(
      autoFactory.resetOdometry("PositionLeftToShoot"),
      autoFactory.trajectoryCmd("PositionLeftToShoot"),
      shootHub(),
      autoFactory.resetOdometry("ShootPositionOneToClimb"),
      new ParallelCommandGroup(
        autoFactory.trajectoryCmd("ShootPositionOneToClimb"),
        new ClimberExtendCommand(m_climberSubsystem)
      ),
      new ClimberClimbCommand(m_climberSubsystem)
    );
  }

  /**
   * Shoots then climbs starting from the center position relative to the drivers
   * @return
   */
  public Command ShootClimbPositionMiddle() {
    return Commands.sequence(
      autoFactory.resetOdometry("PositionMiddleToShoot"),
      autoFactory.trajectoryCmd("PositionMiddleToShoot"),
      shootHub(),
      autoFactory.resetOdometry("ShootPositionTwoToClimb"),
      new ParallelCommandGroup(
        autoFactory.trajectoryCmd("ShootPositionTwoToClimb"),
        new ClimberExtendCommand(m_climberSubsystem)
      ),
      new ClimberClimbCommand(m_climberSubsystem)
    );
  }

  /**
   * Shoots then climbs starting from the right position relative to the drivers
   * @return
   */
  public Command ShootClimbPositionRight() {
    return Commands.sequence(
      autoFactory.resetOdometry("PositionRightToShoot"),
      autoFactory.trajectoryCmd("PositionRightToShoot"),
      shootHub(),
      autoFactory.resetOdometry("ShootPositionThreeToClimb"),
      new ParallelCommandGroup(
        autoFactory.trajectoryCmd("ShootPositionThreeToClimb"),
        new ClimberExtendCommand(m_climberSubsystem)
      ),
      new ClimberClimbCommand(m_climberSubsystem)
    );
  }
  
  public Command getAutonomousCommand() {
    return testAuto();
  }
}
