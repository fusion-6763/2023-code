package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class NavXTurn extends CommandBase {
	private Drive m_DriveSubsystem;

	private double _angle;
	private double current_angle;
	private double ERROR = 2.0; // 2 degrees 
	private double NORMAL_TURN_SPEED = 0.5;
	private double MIN_TURN_SPEED = 0.25;

	/**
	 * Positive angle is a left turn, negative angle is a right turn
	*/
	public NavXTurn(Drive subsys, double angle) {
		m_DriveSubsystem = subsys;
		_angle = angle;
		addRequirements(m_DriveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		_angle += m_DriveSubsystem.getAngle();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		current_angle = m_DriveSubsystem.getAngle();

		double angle_diff = current_angle - _angle;
		// if we're within 15 degrees, then slow us down a bit
		double turning_speed = (Math.abs(angle_diff) < 15) ? MIN_TURN_SPEED : NORMAL_TURN_SPEED;
		if( angle_diff < 0 ) {
			m_DriveSubsystem.tankDrive(turning_speed, -turning_speed);
		} else {
			m_DriveSubsystem.tankDrive(-turning_speed, turning_speed);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		//
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// End the command when we are within the specified threshold
		return Math.abs(current_angle - _angle) < ERROR;
	}
}
