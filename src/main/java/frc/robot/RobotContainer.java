// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoArmRaiseCommand;
import frc.robot.commands.AutoArmLowerCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FollowTagCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.OdometryResetCommand;
import frc.robot.controls.LogitechPro;
//import frc.robot.subsystems.climber.ClimberSubsystem;
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
  // private final ClimberSubsystem m_climberSubsystem;

  private final AutoFactory autoFactory;

  LogitechPro joystick;
  CommandXboxController gamepad;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_fuelSubsystem = new FuelSubsystem();
    m_intakeArmSubsystem = new IntakeArmSubsystem();
    // m_climberSubsystem = new ClimberSubsystem();

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

    Trigger fieldOrientedButton = new Trigger(joystick.button(2));

    Trigger shooterButton = new Trigger(joystick.button(3));
    Trigger manShootButton = new Trigger(joystick.button(4));

    Trigger resetOdometryButton = new Trigger(joystick.button(11));
    Trigger resetGyroButton = new Trigger(joystick.button(12));

    Trigger feedButton = new Trigger(gamepad.x());

    Trigger intakeButton = new Trigger(gamepad.a());
    Trigger intakeOnlyButton = new Trigger(gamepad.y());
    Trigger outtakeButton = new Trigger(gamepad.b());

    // -- Button Assignments --

    manShootButton.whileTrue(m_shooterSubsystem.adjustedSpeedShootCommand());

    feedButton.toggleOnTrue(m_fuelSubsystem.feedCommand());

    intakeButton.toggleOnTrue(m_fuelSubsystem.intakeCommand());
    intakeOnlyButton.toggleOnTrue(m_fuelSubsystem.intakeOnlyCommand());
    outtakeButton.toggleOnTrue(m_fuelSubsystem.outtakeCommand());

    shooterButton.whileTrue(new AutoShootCommand(
        m_shooterSubsystem,
        m_fuelSubsystem, m_swerveSubsystem,
        m_visionSubsystem,
        joystick::getY,
        joystick::getX,
        joystick::getAdjustedThrottle));

    resetOdometryButton.onTrue(new OdometryResetCommand(m_swerveSubsystem, m_visionSubsystem));
    resetGyroButton.onTrue(m_swerveSubsystem.swerveGyroResetCommand());

    m_intakeArmSubsystem.setDefaultCommand(m_intakeArmSubsystem.setArmCommand(() -> gamepad.getLeftY()));

    joystick.button(10).toggleOnTrue(new FollowTagCommand(m_swerveSubsystem, m_visionSubsystem));

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
        joystick::getY,
        joystick::getX,
        joystick::getZ,
        joystick::getAdjustedThrottle,
        fieldOrientedButton.negate()));
  }

  // Autos

  /**
   * Auto for testing purposes
   * 
   * @return
   */
  public Command testAuto() {
    return Commands.sequence(
        autoFactory.resetOdometry("TestPath"),
        autoFactory.trajectoryCmd("TestPath"));
  }

  /**
   * Shoots at the hub
   * 
   * @return
   */
  public Command shootHub() {
    return new WaitCommand(10).raceWith(new AutoShootCommand(m_shooterSubsystem,
        m_fuelSubsystem, m_swerveSubsystem,
        m_visionSubsystem, () -> 0, () -> 0, () -> 0));

  }

  public Command resetGyroCommand() {
    return m_swerveSubsystem.swerveGyroResetCommand();
  }

  // just added these arm commands:

  public Command lowerArm() {
    return new AutoArmLowerCommand(m_intakeArmSubsystem);
  }

  public Command RaiseArm() {
    return new AutoArmRaiseCommand(m_intakeArmSubsystem);
  }

  public Command pickUpFuel() {
    return new Command() {
      /* TODO */
    };
  }

  public enum AutoSwitcher { // enum to switch between different auto modes
    EMPTY_LEFT,
    EMPTY_MIDDLE,
    EMPTY_RIGHT,
    TEST,
    SHOOT_LEFT,
    SHOOT_MIDDLE,
    SHOOT_MIDDLE_LEFT,
    SHOOT_MIDDLE_RIGHT,
    SHOOT_RIGHT,
    SHOOT_RIGHT_THEN_PARK,
    RIGHT_THEN_INTAKE_SHOOT
    // SHOOT_CLIMB_LEFT,
    // SHOOT_CLIMB_MIDDLE,
    // SHOOT_CLIMB_RIGHT,
    // PICKUP_SHOOT_LEFT,
    // PICKUP_SHOOT_CLIMB_LEFT,
    // PICKUP_SHOOT_MIDDLE,
    // PICKUP_SHOOT_CLIMB_MIDDLE,
    // PICKUP_SHOOT_RIGHT,
    // PICKUP_SHOOT_CLIMB_RIGHT
  }

  /**
   * An empty auto just in case
   * 
   * @return An empty command
   */
  public Command EmptyLeft() {
    return Commands.sequence(
        resetGyroCommand(),
        autoFactory.resetOdometry("PositionLeftToShoot"));
  }

  /**
   * An empty auto just in case
   * 
   * @return An empty command
   */
  public Command EmptyMiddle() {
    return Commands.sequence(
        resetGyroCommand(),
        autoFactory.resetOdometry("PositionMiddleToShoot"));
  }

  /**
   * An empty auto just in case
   * 
   * @return An empty command
   */
  public Command EmptyRight() {
    return Commands.sequence(
        resetGyroCommand(),
        autoFactory.resetOdometry("PositionRightToShoot"));
  }

  /**
   * Shoots starting from the left position relative to the drivers
   * 
   * @return
   */
  public Command ShootPositionLeft() {
    return Commands.sequence(
        resetGyroCommand(),
        autoFactory.resetOdometry("PositionLeftToShoot"),
        new WaitCommand(6)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionLeftToShoot")),
        shootHub());
  }

  /**
   * Shoots starting from the left position relative to the drivers
   * 
   * @return
   */
  public Command ShootPositionMiddle() {
    return Commands.sequence(
        resetGyroCommand(),
        new WaitCommand(6)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionMiddleToShoot")),
        shootHub());
  }

  /**
   * Shoots starting from the left position relative to the drivers
   * 
   * @return
   */
  public Command ShootPositionRight() {
    return Commands.sequence(
        new WaitCommand(6)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionRightToShoot")),
        shootHub());
  }

  public Command ShootPositionRightThenPark() {
    return Commands.sequence(
        resetGyroCommand(),
        autoFactory.resetOdometry("PositionRightToShoot"),
        autoFactory.trajectoryCmd("PositionRightToShoot"),

        shootHub(),

        autoFactory.resetOdometry("ShootPositionThreeToHumanPlayerStation"),
        autoFactory.trajectoryCmd("ShootPositionThreeToHumanPlayerStation"));

  }

  /**
   * Shoots starting from the middle position relative to the drivers
   * 
   * @return
   */
  public Command ShootLeftPositionMiddle() {
    return Commands.sequence(
        new WaitCommand(6)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionMiddleToShootLeft")),
        shootHub());
  }

  /**
   * Shoots starting from the middle position relative to the drivers
   * 
   * @return
   */
  public Command ShootRightPositionMiddle() {
    return Commands.sequence(
        new WaitCommand(6)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionMiddleToShootRight")),
        shootHub());
  }

  public Command rightPickupFromFieldAndShoot() {
    return Commands.sequence(

        lowerArm(),
        new WaitCommand(1.29)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionRightToField2", 0)),

        Commands.deadline(
            new WaitCommand(2.59 - 1.29)
                .raceWith(
                    new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionRightToField2", 1)),
            m_fuelSubsystem.intakeCommand(),
            m_intakeArmSubsystem.setArmCommand(() -> 0.1)),

        new WaitCommand(5.05 - 2.59)
            .raceWith(new FollowTrajectoryCommand(m_swerveSubsystem, m_visionSubsystem, "PositionRightToField2", 2)),

        shootHub());
  }

  /**
   * Returns the autonomous that was chosen from the dash
   * 
   * @param autoMode
   * @return Command
   */
  public Command getAutonomousCommand(AutoSwitcher autoMode) {
    switch (autoMode) {
      default:
      case EMPTY_LEFT:
        return EmptyLeft();
      case EMPTY_MIDDLE:
        return EmptyMiddle();
      case EMPTY_RIGHT:
        return EmptyRight();
      case TEST:
        return testAuto();
      case SHOOT_LEFT:
        return ShootPositionLeft();
      case SHOOT_MIDDLE:
        return ShootPositionMiddle();
      case SHOOT_MIDDLE_LEFT:
        return ShootLeftPositionMiddle();
      case SHOOT_MIDDLE_RIGHT:
        return ShootRightPositionMiddle();
      case SHOOT_RIGHT:
        return ShootPositionRight();
      case SHOOT_RIGHT_THEN_PARK:
        return ShootPositionRightThenPark();
      case RIGHT_THEN_INTAKE_SHOOT:
        return rightPickupFromFieldAndShoot();
      // case OWEN:
      // return OwenCommand();
    }
  }
}
