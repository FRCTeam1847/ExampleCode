// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardCommand extends Command {
  private final DriveTrain m_driveTrain;
  private double maxTime = 0.0;
  private Timer localTimer = new Timer();
  /** Creates a new DriveForwardCommand that will run forward for the timeLimit provided
   * 
   * @param driveTrain
   * @param timeLimit
   */
  public DriveForwardCommand(DriveTrain driveTrain, double timeLimit) {
    m_driveTrain = driveTrain;
    maxTime=timeLimit;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Reset and start timer */
    localTimer.reset();
    localTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (localTimer.get() < maxTime) {
      m_driveTrain.drive.arcadeDrive(0.2, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
