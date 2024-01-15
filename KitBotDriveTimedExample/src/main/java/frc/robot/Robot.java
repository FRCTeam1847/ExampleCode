// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
   * Define Auto Selector.
   * This is helpful when choosing a different auto for certain situations.
   */
  private static final String Auto1 = "Default";
  private static final String Auto2 = "3 seconds";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // Define timer for auto
  private Timer timer = new Timer();

  // Define JoyStick. This will be the joystick plugged in to the first port.
  private final Joystick joystick = new Joystick(0);

  /**
   * Drive Motors are defined here with the their CAN ID as the number.
   * In the case there is only 1 drive motor on each side (2 total motors) you can
   * comment out the second motor here.
   */
  private final WPI_TalonSRX m_left1 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_left2 = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_right1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_right2 = new WPI_TalonSRX(3);

  // Define the motors in a Differential Drive provided by the library.
  DifferentialDrive drive = new DifferentialDrive(m_left1, m_right1);

  @Override
  public void robotInit() {
    /** Set up auto chooser on smart dashboard */
    m_chooser.setDefaultOption(Auto1, Auto1);
    m_chooser.addOption(Auto2, Auto2);
    SmartDashboard.putData("Auto choices", m_chooser);

    /**
     * Reset motors to default configuration and stop the motors for safety.
     */
    m_left1.configFactoryDefault();
    m_left2.configFactoryDefault();
    m_right1.configFactoryDefault();
    m_right2.configFactoryDefault();
    m_left1.stopMotor();
    m_left2.stopMotor();
    m_right1.stopMotor();
    m_right2.stopMotor();

    /**
     * Set up follower motors.
     */
    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

  }

  @Override
  public void autonomousInit() {
    /* Reset and start timer */
    timer.reset();
    timer.start();
    /* Get selected auto */
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case Auto1:
        if (timer.get() < 2) {
          drive.arcadeDrive(0.2, 0, false);
        }
        break;
      case Auto2:
        if (timer.get() < 3) {
          drive.arcadeDrive(0.2, 0, false);
        }
        break;
      default:
        break;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /**
     * Give the drive its values from the controller.
     */
    drive.arcadeDrive(joystick.getX(), joystick.getY());
  }
}
