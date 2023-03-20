// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/** An example command that uses an example subsystem. */
public class PathPlannerAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;
  private final Intake intake;

  private static final double ksVolts = 0.18725;
  private static final double kvVoltSecondsPerMeter = 0.13029;
  private static final double kaVoltSecondsSquaredPerMeter = 0.020664;
  private static final double kPDriveVel = 3.1867E-08;

  public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(0.63); // measured 63 cm from middle to middle
  private static final double kMaxSpeedMetersPerSecond = 1;
  private static final double kMaxAccelMetersPerSecond = 1;
  private static final double kRamseteB = 2;
  private static final double kRamseteZeta = 0.7;

  DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
      ksVolts,
      kvVoltSecondsPerMeter,
      kaVoltSecondsSquaredPerMeter),
    kDriveKinematics,
    10);

  TrajectoryConfig config = new TrajectoryConfig(
    kMaxSpeedMetersPerSecond,
    kMaxAccelMetersPerSecond
  )
  .setKinematics(kDriveKinematics)
  .addConstraint(autoVoltageConstraint);

  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  new Pose2d(0,0, new Rotation2d(0)),
   List.of(new Translation2d(1,1), new Translation2d(2,-1)),
   new Pose2d(0,0, new Rotation2d(0)),
   config);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathPlannerAuto(Drive _drive, Intake _intake) {
    drive = _drive;
    intake = _intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(exampleTrajectory.getInitialPose());
    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory, 
      drive::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
      kDriveKinematics, 
      drive::getWheelSpeeds, 
      new PIDController(kPDriveVel,0, 0),
      new PIDController(kPDriveVel,0, 0), 
      drive::tankDriveVolts,
      drive
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
