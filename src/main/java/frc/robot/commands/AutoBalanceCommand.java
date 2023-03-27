package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalanceCommand extends CommandBase {
	private Drive m_DriveSubsystem;

	private double error;
	private double currentAngle;
	private double drivePower;

	/** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balance */
	public AutoBalanceCommand(Drive subsys) {
		m_DriveSubsystem = subsys;
		addRequirements(m_DriveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Uncomment the line below this to simulate the gyroscope axis with a controller joystick
		// Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
		currentAngle = m_DriveSubsystem.getPitch();

		error = Constants.MachineConstants.beamBalancedGoalDegrees - currentAngle;
		drivePower = -Math.min(Constants.MachineConstants.beamBalancedDriveKp * error, 1);

		// Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
		if (drivePower < 0) {
			drivePower *= Constants.MachineConstants.autoBalanceScalarFrontBackDiff;
		}

		// Limit the max power
		double MAX_POWER = 0.3;
		if (Math.abs(drivePower) > MAX_POWER) {
			drivePower = Math.copySign(MAX_POWER, drivePower);
		}
		
		m_DriveSubsystem.tankDrive(drivePower, drivePower);

		// Debugging Print Statments
		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_DriveSubsystem.tankDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
		return Math.abs(error) < Constants.MachineConstants.beamBalancedAngleThresholdDegrees;
	}
}
