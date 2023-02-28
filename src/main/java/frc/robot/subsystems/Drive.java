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
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  private static CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
  private static CANSparkMax motor3 = new CANSparkMax(3, MotorType.kBrushless);
  private static CANSparkMax motor4 = new CANSparkMax(4, MotorType.kBrushless);

  private static MotorControllerGroup rightGroup = new MotorControllerGroup(motor1, motor2);
  private static MotorControllerGroup leftGroup = new MotorControllerGroup(motor3, motor4);
  private static DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

  private DifferentialDriveOdometry m_odometry;
  private final AHRS ahrs = new AHRS(); 
  private RelativeEncoder encoderRight = motor2.getEncoder();
  private RelativeEncoder encoderLeft = motor4.getEncoder();

  public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(0.63); // measured 63 cm from middle to middle
  // public final MecanumDriveKinematics mecanumDriveKinematics =
  //       new MecanumDriveKinematics(
  //         new Translation2d(0, 0.2921), // 12.5" is .2921m
  //         new Translation2d(0, -0.2921),
  //         new Translation2d(0, 0.2921),
  //         new Translation2d(0, -0.2921)
  //       );
  
  public Drive() {
    // leftGroup.setInverted(true);
    
    // rightGroup.setInverted(true);
    //differentialDrive.setSafetyEnabled(false);
    // this.setDefaultCommand(Commands.run(
    //   () ->
    //   differentialDrive.arcadeDrive(0, 0),
    //   this
    // ));
    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
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
        });
  }

  public void customDrive (double forward, double rotation) {
      differentialDrive.arcadeDrive(forward, rotation);
  }



  

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
    m_odometry.update(ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
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
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity());
  }

  // public MecanumDriveWheelSpeeds getMechanumWheelSpeeds() {
  //   return new MecanumDriveWheelSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity(),
  //                                       encoderLeft.getVelocity(), encoderRight.getVelocity());
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
      ahrs.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
  }

  //public void straightRampUpDrive(double slowTime, double fastTime){
  ////  double BACKWARDS = 1.0;
 //   customDrive(0, BACKWARDS * 0.4), drive).withTimeout(slowTime);
  //  customDrive(0, BACKWARDS * 0.4), drive).withTimeout(fastTime);
 // }
}
