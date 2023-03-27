// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForwardDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;
  private double _speed;
  private double _distance;
  private double start_angle;
  private double encoder_start;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwardDistance(Drive _drive, double speed, double distance) {
    drive = _drive;
    _speed = speed;
    _distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drive.customDrive(0.25, 0);
    start_angle = drive.getYaw();
    drive.resetEncoders();
    //encoder_start = Math.abs(drive.getLeftEncoder().getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing forward...");
    double angle_error = start_angle - drive.getYaw();
    double l1 = Math.min(_speed + angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);
    double l2 = Math.min(_speed - angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);

    drive.tankDrive(l1, l2);
    //drive.arcadeDrive(0.4, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(0 + " __ " + drive.getLeftEncoder().getPosition());
    if (_distance < Math.abs(drive.getLeftEncoder().getPosition()) ) {
      return true;
    }
    return false;
  }
}
