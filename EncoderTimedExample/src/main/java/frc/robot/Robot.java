// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * Define motor controller
   */
  private WPI_TalonSRX m_motor = new WPI_TalonSRX(5);

  /** Define joystick */
  private final Joystick joystick = new Joystick(0);

  /** Define values for TalonSRX PID */
  static final int kSlotIdx = 0;
  static final int kPIDLoopIdx = 0;
  static final int kTimeoutMs = 30;
  static boolean kSensorPhase = true;
  static boolean kMotorInvert = false;
  static double RPM = 500;
  static int unitsPer100ms = 600;

  /** Define Gains. Adjust these values to tune the accuracy of the mechanism.  */
  static final Gains kGains = new Gains(0.1, 0.0, 1.0, 0.0, 0, 0.25);

  /** Define max rotations */
  static int fullRotation = 4096;
  static double kDegreeConstant = fullRotation/360;;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Factory Default motor */
    m_motor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        kPIDLoopIdx,
        kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    m_motor.setSensorPhase(kSensorPhase);

    /* Config the peak and nominal outputs, 12V means full */
    m_motor.configNominalOutputForward(0, kTimeoutMs);
    m_motor.configNominalOutputReverse(0, kTimeoutMs);
    m_motor.configPeakOutputForward(kGains.kPeakOutput, kTimeoutMs);
    m_motor.configPeakOutputReverse(-kGains.kPeakOutput, kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    m_motor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_motor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    m_motor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    m_motor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    m_motor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute
     * position, and intitally set the relative sensor to match.
     */
    int absolutePosition = m_motor.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (kSensorPhase) {
      absolutePosition *= -1;
    }
    if (kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    m_motor.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /** Display encoder value on dashboard */
    SmartDashboard.putNumber("Encoder Value", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Angle", m_motor.getSelectedSensorPosition()/ kDegreeConstant);
    SmartDashboard.putNumber("Velocity", m_motor.getSelectedSensorVelocity());
  }

 
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Set up buttons
    if (joystick.getRawButton(1)) {
      /** Move motor to 0 Degrees */
      m_motor.set(ControlMode.Position, 0);
    } else if (joystick.getRawButton(2)) {
      /** Move motor to 180 Degrees */
      m_motor.set(ControlMode.Position, fullRotation / 2);
    } else if (joystick.getRawButton(3)) {
      /** Move motor to 360 Degrees */
      m_motor.set(ControlMode.Position, fullRotation);
    }
    else {
      /**
			 * Convert RPM to units / 100ms.
			 * 4096 Units/Rev * '500' RPM / 600 100ms/min in either direction:
			 * velocity set point is in units/100ms
			 */
      double targetVelocity_UnitsPer100ms = -joystick.getY() * RPM * fullRotation / unitsPer100ms;
			/* 500 RPM in either direction */
			m_motor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
