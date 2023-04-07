// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
	private static CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
	private static CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
	private static CANSparkMax motor3 = new CANSparkMax(3, MotorType.kBrushless);
	private static CANSparkMax motor4 = new CANSparkMax(4, MotorType.kBrushless);

	private static MotorControllerGroup rightGroup = new MotorControllerGroup(motor1, motor2);
	private static MotorControllerGroup leftGroup = new MotorControllerGroup(motor3, motor4);
  private static DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

  //private DifferentialDriveOdometry m_odometry;
  private final AHRS ahrs = new AHRS(); 
  private RelativeEncoder encoderRight = motor2.getEncoder();
  private RelativeEncoder encoderLeft = motor4.getEncoder();

  private static final double ksVolts = 0;
  private static final double kvVoltSecondsPerMeter = 0;
  private static final double kaVoltSecondsSquaredPerMeter = 0.2;
  private static final double kPDriveVel = 0;

  public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(0.63); // measured 63 cm from middle to middle
  private static final double kMaxSpeedMetersPerSecond = 1;
  private static final double kMaxAccelMetersPerSecond = 1;
  
  public Drive() {
    leftGroup.setInverted(false);
    rightGroup.setInverted(true);
    
    encoderLeft.setPositionConversionFactor(2.3);
    encoderRight.setPositionConversionFactor(2.3);
    
    //differentialDrive.setSafetyEnabled(false);
    // this.setDefaultCommand(Commands.run(
    //   () ->
    //   differentialDrive.arcadeDrive(0, 0),
    //   this
    // ));
    //m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
  }

	private static Translation2d location = new Translation2d(1.88, 0.34);

	public void customDrive (double rotation, double forward) {
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
	public double getPitch() { return ahrs.getPitch(); }
	public double getRoll() { return ahrs.getRoll(); } // SHOULD ALWAYS BE NEAR 0
	public double getYaw() { return ahrs.getYaw(); }
  public double getAngle() { return ahrs.getAngle(); }

	/**
	 * Series of functions to return velocity from the gyro. The gyro does not claim any
	 * meaningful precision from this, and it should be periodically recalibrated based on
	 * markers in the field.
	 * 
	 * @return double for each velocity vector component
	 */
	public double GetVelX() { return ahrs.getVelocityX(); }
	public double GetVelY() { return ahrs.getVelocityY(); }
	public double GetVelZ() { return ahrs.getVelocityZ(); }


  /**
   * Returns the encoders, this is for autonomous commands
   */
  public RelativeEncoder getLeftEncoder() { return encoderLeft; }
  public RelativeEncoder getRightEncoder() { return encoderRight; }

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
    //m_odometry.update(ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
    SmartDashboard.putNumber("encoderLeft", encoderLeft.getPosition());
    SmartDashboard.putNumber("encoderRight", encoderRight.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return null;
    //return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  //   resetEncoders();
  //   m_odometry.resetPosition(
  //     ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);
  // }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    System.out.println("Encoders reset");
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts/3);
    rightGroup.setVoltage(rightVolts/3);
    differentialDrive.feed();
  }
}
