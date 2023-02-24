// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
//import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive(); // Robot wheels
  private final Intake intake = new Intake();
  private final PowerDistribution powerDistribution = new PowerDistribution(); // PDP / PDB

  // Controller for the driver
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final double speed = 0.75;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Clear sticky faults, AKA the yellow flashing on the PDP
    powerDistribution.clearStickyFaults();

    // Sets the drive's default command to "teleop driving"
    drive.setDefaultCommand(Commands.run(
        () -> drive.customDrive(m_driverController.getRawAxis(2) * speed, m_driverController.getRawAxis(1) * speed),
        drive));

    // sets the intake's default command to stop running.
    intake.setDefaultCommand(Commands.run(() -> intake.neutral(), intake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // TEMP Intake on left trigger
    m_driverController.button(5).whileTrue(Commands.run(() -> intake.forward(), intake));

    // TEMP Outtake of right trigger
    m_driverController.button(6).whileTrue(Commands.run(() -> intake.backward(), intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(3, 3));    
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("IntakeOn", Commands.run(() -> System.out.println("IntakeOn")));
    eventMap.put("IntakeOff", Commands.run(() -> System.out.println("IntakeOff")));
    
    RamseteController rController = new RamseteController();
    RamseteAutoBuilder ramseteCommand =
          new RamseteAutoBuilder( drive::getPose,
                                  drive::resetOdometry,
                                  rController,
                                  drive.kDriveKinematics,
                                  drive::tankDriveVolts,
                                  eventMap,
                                  drive);
    return ramseteCommand.fullAuto(examplePath); // Autos.exampleAuto(m_exampleSubsystem);
  }

  public void teleopInit() {

  }
}
