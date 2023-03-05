// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
	private static CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
	private static CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
	private static CANSparkMax motor3 = new CANSparkMax(3, MotorType.kBrushless);
	private static CANSparkMax motor4 = new CANSparkMax(4, MotorType.kBrushless);

	private static MotorControllerGroup rightGroup = new MotorControllerGroup(motor1, motor2);
	private static MotorControllerGroup leftGroup = new MotorControllerGroup(motor3, motor4);

	private static AHRS gyro = new AHRS();

	private static DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

	private static Translation2d location = new Translation2d(1.88, 0.34);

	public Drive() {
		// leftGroup.setInverted(true);

		// rightGroup.setInverted(true);
		//differentialDrive.setSafetyEnabled(false);
		// this.setDefaultCommand(Commands.run(
		//   () ->
		//   differentialDrive.arcadeDrive(0, 0),
		//   this
		// ));
		gyro.calibrate();
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public CommandBase myDefaultCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return run(
			() -> {
				System.out.println("help");
				customDrive(0, 0);
			}
		);
	}

	public void customDrive (double forward, double rotation) {
		arcadeDrive(forward, rotation);
	}
	public void arcadeDrive(double forward, double rotation) {
		differentialDrive.arcadeDrive(forward, rotation);
	}
	public void tankDrive(double left, double right) {
		// NOTE: In tank drive, both left and right should be positive for forwards per the API
		differentialDrive.tankDrive(left, right);
	}


	/**
	 * Metrics of the robots current state
	 * 
	 * Since our robot is extremely close to the ground and stable, our roll should be near 0 and
	 * irrelevant for AI purposes. Likely to only happen when 90 degrees on the balance beam.
	 * 
	 * @return float of the rotation
	 */
	public double getPitch() { return gyro.getPitch(); }
	public double getRoll() { return gyro.getRoll(); } // SHOULD ALWAYS BE NEAR 0
	public double getYaw() { return gyro.getYaw(); }

	/**
	 * Series of functions to return velocity from the gyro. The gyro does not claim any
	 * meaningful precision from this, and it should be periodically recalibrated based on
	 * markers in the field.
	 * 
	 * @return double for each velocity vector component
	 */
	public double GetVelX() { return gyro.getVelocityX(); }
	public double GetVelY() { return gyro.getVelocityY(); }
	public double GetVelZ() { return gyro.getVelocityZ(); }



	/**
	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
