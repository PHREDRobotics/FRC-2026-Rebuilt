// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.GoToPoseCommand;
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

    Trigger shooterButton = new Trigger(joystick.button(6));
    Trigger manShootButton = new Trigger(joystick.button(4));

    Trigger intakeButton = new Trigger(gamepad.a());

    Trigger armUpButton = new Trigger(gamepad.start());
    Trigger armDownButton = new Trigger(gamepad.back());

    Trigger climberClimbButton = new Trigger(gamepad.povLeft());
    Trigger climberExtendButton = new Trigger(gamepad.povUp());
    Trigger climberRetractButton = new Trigger(gamepad.povDown());

    // -- Button Assignments --

    shooterButton.whileTrue(new AutoShootCommand(m_shooterSubsystem, m_fuelSubsystem, m_swerveSubsystem,
        m_visionSubsystem, joystick::getX, joystick::getY));
    // manShootButton.whileTrue(m_shooterSubsystem.shootCommand(() ->
    // Constants.ShooterConstants.kInitialShootingSpeed));

    // joystick.button(3).whileTrue(new RunCommand(m_fuelSubsystem::feed,
    // m_fuelSubsystem));

    // intakeButton.toggleOnTrue(m_fuelSubsystem.intakeCommand());

    // armUpButton.onTrue(m_intakeArmSubsystem.raiseIntakeCommand());
    // armDownButton.onTrue(m_intakeArmSubsystem.lowerIntakeCommand());

    // climberClimbButton.onTrue(m_climberSubsystem.climbCommand());
    // climberExtendButton.onTrue(m_climberSubsystem.extendCommand());
    // climberRetractButton.onTrue(m_climberSubsystem.retractCommand());

    // -- Default commands --

    joystick.button(1).onTrue(new GoToPoseCommand(m_swerveSubsystem, m_visionSubsystem, new Pose2d()));

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
        joystick::getCoolerY,
        joystick::getX,
        joystick::getCoolerZ,
        joystick::getAdjustedThrottle,
        joystick.button(1)));
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
    return new WaitCommand(2).raceWith(new AutoShootCommand(m_shooterSubsystem, m_fuelSubsystem, m_swerveSubsystem,
        m_visionSubsystem, () -> 0, () -> 0));
  }

  public Command pickUpFuel() {
    return new Command() {
      /* TODO */
    };
  }

  public enum AutoSwitcher { // enum to switch between different auto modes
    TEST,
    SHOOT_CLIMB_LEFT,
    SHOOT_CLIMB_MIDDLE,
    SHOOT_CLIMB_RIGHT,
    PICKUP_SHOOT_LEFT,
    PICKUP_SHOOT_CLIMB_LEFT,
    PICKUP_SHOOT_MIDDLE,
    PICKUP_SHOOT_CLIMB_MIDDLE,
    PICKUP_SHOOT_RIGHT,
    PICKUP_SHOOT_CLIMB_RIGHT
  }

  /**
   * Shoots then climbs starting from the left position relative to the drivers
   * 
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
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
  }

  /**
   * Shoots then climbs starting from the center position relative to the drivers
   * 
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
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
  }

  /**
   * Shoots then climbs starting from the right position relative to the drivers
   * 
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
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
  }

  /**
   * Starts on the left then gets more fuel from the spot to the left of driver's
   * view then shoots
   * 
   * @return
   */
  public Command PickupAndShootLeft() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionLeftToPickupLeft"),
        autoFactory.trajectoryCmd("PositionLeftToPickupLeft"),
        pickUpFuel(),
        autoFactory.resetOdometry("PickupLeftToShootLeft"),
        autoFactory.trajectoryCmd("PickupLeftToShootLeft"),
        shootHub());
  }

  /**
   * Starts on the left then gets more fuel from the spot to the left of driver's
   * view then shoots and then climbs
   * 
   * @return
   */
  public Command PickupAndShootLeftAndClimb() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionLeftToPickupLeft"),
        autoFactory.trajectoryCmd("PositionLeftToPickupLeft"),
        pickUpFuel(),
        autoFactory.resetOdometry("PickupLeftToShootLeft"),
        autoFactory.trajectoryCmd("PickupLeftToShootLeft"),
        shootHub(),
        autoFactory.resetOdometry("ShootPositionOneToClimb"),
        new ParallelCommandGroup(
            autoFactory.trajectoryCmd("ShootPositionOneToClimb"),
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
  }

  /**
   * Starts in the middle then gets more fuel from the spot to the left of
   * driver's view then shoots
   * 
   * @return
   */
  public Command PickupAndShootMiddle() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionMiddleToPickup"),
        autoFactory.trajectoryCmd("PositionMiddleToPickup"),
        pickUpFuel(),
        autoFactory.resetOdometry("PickupLeftToShootMiddle"),
        autoFactory.trajectoryCmd("PickupLeftToShootMiddle"),
        shootHub());
  }

  /**
   * Starts in the middle then gets more fuel from the spot to the left of
   * driver's view then shoots then climbs
   * 
   * @return
   */
  public Command PickupAndShootMiddleAndClimb() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionMiddleToPickup"),
        autoFactory.trajectoryCmd("PositionMiddleToPickup"),
        pickUpFuel(),
        autoFactory.resetOdometry("PickupLeftToShootMiddle"),
        autoFactory.trajectoryCmd("PickupLeftToShootMiddle"),
        shootHub(),
        autoFactory.resetOdometry("ShootPositionTwoToClimb"),
        new ParallelCommandGroup(
            autoFactory.trajectoryCmd("ShootPositionTwoToClimb"),
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
  }

  /**
   * Starts on the right then gets more fuel from the human player station then
   * shoots
   * 
   * @return
   */
  public Command PickupAndShootRight() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionRightToHumanPlayerPickup"),
        autoFactory.trajectoryCmd("PositionRightToHumanPlayerPickup"),
        pickUpFuel(),
        autoFactory.resetOdometry("HumanPlayerToShootPositionThree"),
        autoFactory.trajectoryCmd("HumanPlayerToShootPositionThree"),
        shootHub());
  }

  /**
   * Starts on the right then gets more fuel from the human player station then
   * shoots and then climbs
   * 
   * @return
   */
  public Command PickupAndShootRightAndClimb() {
    return Commands.sequence(
        autoFactory.resetOdometry("PositionRightToHumanPlayerPickup"),
        autoFactory.trajectoryCmd("PositionRightToHumanPlayerPickup"),
        pickUpFuel(),
        autoFactory.resetOdometry("HumanPlayerToShootPositionThree"),
        autoFactory.trajectoryCmd("HumanPlayerToShootPositionThree"),
        shootHub(),
        autoFactory.resetOdometry("ShootPositionThreeToClimb"),
        new ParallelCommandGroup(
            autoFactory.trajectoryCmd("ShootPositionThreeToClimb"),
            m_climberSubsystem.extendCommand()),
        m_climberSubsystem.climbCommand());
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
      case TEST:
        return testAuto();

      case SHOOT_CLIMB_LEFT:
        return ShootClimbPositionLeft();

      case SHOOT_CLIMB_MIDDLE:
        return ShootClimbPositionMiddle();

      case SHOOT_CLIMB_RIGHT:
        return ShootClimbPositionRight();

      case PICKUP_SHOOT_LEFT:
        return PickupAndShootLeft();

      case PICKUP_SHOOT_CLIMB_LEFT:
        return PickupAndShootLeftAndClimb();

      case PICKUP_SHOOT_MIDDLE:
        return PickupAndShootMiddle();

      case PICKUP_SHOOT_CLIMB_MIDDLE:
        return PickupAndShootMiddleAndClimb();

      case PICKUP_SHOOT_RIGHT:
        return PickupAndShootRight();

      case PICKUP_SHOOT_CLIMB_RIGHT:
        return PickupAndShootRightAndClimb();
    }
  }
}
