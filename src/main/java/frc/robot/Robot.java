// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final PowerDistribution m_pdh;

  private final SendableChooser<RobotContainer.AutoSwitcher> autoChooser = new SendableChooser<>();

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void robotInit() {
    autoChooser.setDefaultOption("Test-Auto", RobotContainer.AutoSwitcher.TEST);

    autoChooser.addOption("Left-Shoot-Then-Climb", RobotContainer.AutoSwitcher.SHOOT_CLIMB_LEFT);
    autoChooser.addOption("Middle-Shoot-Then-Climb", RobotContainer.AutoSwitcher.SHOOT_CLIMB_MIDDLE);
    autoChooser.addOption("Right-Shoot-Then-Climb", RobotContainer.AutoSwitcher.SHOOT_CLIMB_RIGHT);

    autoChooser.addOption("Left-Pickup-Then-Shoot", RobotContainer.AutoSwitcher.PICKUP_SHOOT_LEFT);
    autoChooser.addOption("Left-Pickup-Then-Shoot-Then-Climb", RobotContainer.AutoSwitcher.PICKUP_SHOOT_CLIMB_LEFT);

    autoChooser.addOption("Middle-Pickup-Then-Shoot", RobotContainer.AutoSwitcher.PICKUP_SHOOT_MIDDLE);
    autoChooser.addOption("Middle-Pickup-Then-Shoot-Then-Climb", RobotContainer.AutoSwitcher.PICKUP_SHOOT_CLIMB_MIDDLE);

    autoChooser.addOption("Right-Pickup-Then-Shoot", RobotContainer.AutoSwitcher.PICKUP_SHOOT_RIGHT);
    autoChooser.addOption("Right-Pickup-Then-Shoot-Then-Climb", RobotContainer.AutoSwitcher.PICKUP_SHOOT_CLIMB_RIGHT);

        SmartDashboard.putData("Auto mode", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Amps/Total", m_pdh.getTotalCurrent());

    SmartDashboard.putNumber("Amps/FRdrive", m_pdh.getCurrent(16));
    SmartDashboard.putNumber("Amps/FRturn", m_pdh.getCurrent(17));
    SmartDashboard.putNumber("Amps/FLdrive", m_pdh.getCurrent(18));
    SmartDashboard.putNumber("Amps/FLturn", m_pdh.getCurrent(19));
    SmartDashboard.putNumber("Amps/BRdrive", m_pdh.getCurrent(3));
    SmartDashboard.putNumber("Amps/BRturn", m_pdh.getCurrent(2));
    SmartDashboard.putNumber("Amps/BLdrive", m_pdh.getCurrent(14));
    SmartDashboard.putNumber("Amps/BLturn", m_pdh.getCurrent(0));

    SmartDashboard.putNumber("Amps/leftFeeder", m_pdh.getCurrent(13));
    SmartDashboard.putNumber("Amps/rightFeeder", m_pdh.getCurrent(15));

    SmartDashboard.putNumber("Amps/hopperFloor", m_pdh.getCurrent(12));
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public double getTotalAmps() {
    NetworkTable ampsTable = NetworkTableInstance.getDefault().getTable("Amps");
    double total = 0;

    for (String key : ampsTable.getKeys()) {
      total += ampsTable.getEntry(key).getDouble(0.0);
    }

    return total;
  }
}
