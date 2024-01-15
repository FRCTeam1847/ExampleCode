// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {

   /**
   * Drive Motors are defined here with the their CAN ID as the number.
   * In the case there is only 1 drive motor on each side (2 total motors) you can
   * comment out the second motor here.
   * Note: Motor constants are defined in the Constants.java file
   */
  private final WPI_TalonSRX m_left1 = new WPI_TalonSRX(OperatorConstants.kLeftMotor1);
  private final WPI_TalonSRX m_left2 = new WPI_TalonSRX(OperatorConstants.kLeftMotor2);
  private final WPI_TalonSRX m_right1 = new WPI_TalonSRX(OperatorConstants.kRightMotor1);
  private final WPI_TalonSRX m_right2 = new WPI_TalonSRX(OperatorConstants.kRightMotor2);

  // Define the motors in a Differential Drive provided by the library.
  public DifferentialDrive drive = new DifferentialDrive(m_left1, m_right1);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
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
  public void periodic() {
    // This method will be called once per scheduler run
    drive.arcadeDrive(RobotContainer.m_driverController.getX(), RobotContainer.m_driverController.getY());
  }
}
