// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Define the drive train subsystem */
  private final DriveTrain m_DriveTrain = new DriveTrain();

  /*
   * Define Auto Selector.
   * This is helpful when choosing a different auto for certain situations.
   */
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  /** 
   * Define the commands to run.
   */
   private final Command m_DriveFor2Sec = new DriveForwardCommand(m_DriveTrain, 2);
   private final Command m_DriveFor3Sec = new DriveForwardCommand(m_DriveTrain, 3);

  // Define timer for auto
  public static Timer m_timer = new Timer();

  /** Define JoyStick. This will be the joystick plugged in to the first port. */
  public static Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /** Set up auto chooser on smart dashboard */
    m_chooser.setDefaultOption("Drive 2 Seconds", m_DriveFor2Sec);
    m_chooser.addOption("Drive 3 Seconds", m_DriveFor3Sec);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
