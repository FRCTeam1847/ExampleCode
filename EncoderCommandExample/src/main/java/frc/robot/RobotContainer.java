// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleMotor;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

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
  // The robot's subsystems and commands are defined here...
  private final ExampleMotor m_exampleMotorSubsystem = new ExampleMotor();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Set up buttons to trigger setAngle commands.
    m_driverController.button(1).whileTrue(m_exampleMotorSubsystem.setAngle(0));
    m_driverController.button(2).whileTrue(m_exampleMotorSubsystem.setAngle(180));
    m_driverController.button(3).whileTrue(m_exampleMotorSubsystem.setAngle(360));
  }
}
