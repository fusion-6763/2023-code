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
  private boolean scale_speed; // defaults to TRUE

  private final double MIN_SPEED = 0.3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveBackwardDistance(Drive _drive, double speed, double distance) {
    drive = _drive;
    _speed = speed;
    _distance = distance;
	scale_speed = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  public DriveBackwardDistance setSpeedScaling(boolean to_scale) {
	scale_speed = to_scale;
	return this;
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
	double effective_speed = _speed;
	// linear scaling based on encoder distance
	// this actually leads to exponential acceleration over the distance
	// since the faster speed leads to a faster distance delta
	if (scale_speed) {
		// this is vaguely what we're doing
		// https://www.desmos.com/calculator/8qx3tvsnd3
		double upscale_distance = 15; // we scale up over 4 inches
		double slowdown_distance = 15; // we slow down over 3 inches
		double current_distance = Math.abs(drive.getLeftEncoder().getPosition());

		double start_scaling = MIN_SPEED + (_speed-MIN_SPEED) * Math.min(current_distance / upscale_distance, 1.0);
		// the way this works, is we add the slowdown distance to the current distance, anything that hangs "over" the end we shove into the scaling
		// so for most of the range we are under 0, but once we reach the last slowdown_distance of units, start to reduce the speed
		double end_scaling = _speed - (_speed-MIN_SPEED) * Math.max((current_distance + slowdown_distance - _distance)/slowdown_distance, 0.0);

		effective_speed = Math.min(start_scaling, end_scaling);
	}
  System.out.println("Driving Backwards at " + effective_speed);

    double angle_error = start_angle - drive.getYaw();
    double l1 = Math.min(effective_speed - angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);
    double l2 = Math.min(effective_speed + angle_error * Constants.MachineConstants.straightCorrectionCoeff, 0.98);

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
