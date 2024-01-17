// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gains;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

public class ExampleMotor extends SubsystemBase {

  /**
   * Define motor controller
   */
  private WPI_TalonSRX m_motor = new WPI_TalonSRX(OperatorConstants.kMotorID);

  /** Define values for Talon PID */
  static final int kSlotIdx = 0;
  static final int kPIDLoopIdx = 0;
  static final int kTimeoutMs = 30;
  static boolean kSensorPhase = true;
  static boolean kMotorInvert = false;
  static double RPM = 500;
  static int unitsPer100ms = 600;

  /** Define Gains. Adjust these values to tune the accuracy of the mechanism. */
  static final Gains kGains = new Gains(0.1, 0.0, 1.0, 0.0, 0, 0.25);

  /** Define max rotations */
  static int fullRotation = 4096;
  static double kDegreeConstant = fullRotation / 360;;

  /** Creates a new ExampleMotor. */
  public ExampleMotor() {
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
     * position, and initially set the relative sensor to match.
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

  public void setMotorVelocity(double input) {
    /**
     * Convert RPM to units / 100ms.
     * 4096 Units/Rev * '500' RPM / 600 100ms/min in either direction:
     * velocity set point is in units/100ms
     */
    double targetVelocity_UnitsPer100ms = -input * RPM * fullRotation / unitsPer100ms;
    /* 500 RPM in either direction */
    m_motor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command setAngle(int angle) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          /* one-time action goes here */
          double encoderValue = angle * kDegreeConstant;
          m_motor.set(ControlMode.Position, encoderValue);
        });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void periodic() {
    /** Display encoder value on dashboard */
    SmartDashboard.putNumber("Encoder Value", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Angle", m_motor.getSelectedSensorPosition() / kDegreeConstant);
    SmartDashboard.putNumber("Velocity", m_motor.getSelectedSensorVelocity());
    setMotorVelocity(RobotContainer.m_driverController.getX());
  }
}
