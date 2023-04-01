// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveBackwardDistance extends CommandBase {
  private final Drive drive;
  private double _speed;
  private double _distance;
  private double start_angle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveBackwardDistance(Drive _drive, double speed, double distance) {
    drive = _drive;
    _speed = speed;
    _distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_angle = drive.getYaw();
    drive.resetEncoders();
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing backward...");
    double angle_error = start_angle - drive.getYaw();
    double l1 = Math.min(_speed + angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);
    double l2 = Math.min(_speed - angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);

    drive.tankDrive(-l1, -l2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_distance < Math.abs(drive.getLeftEncoder().getPosition()) ) {
      return true;
    }
    return false;
  }
}
