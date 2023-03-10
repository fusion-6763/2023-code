// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CustomCommand;
import frc.robot.commands.DriveForward;
//import frc.robot.commands.Autos;
//import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

import java.util.HashMap;

import javax.swing.plaf.basic.BasicComboBoxUI.FocusHandler;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  // public static Boolean isSlower = false;

  // Controller for the driver
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final Joystick vroomstick = new Joystick(Constants.OperatorConstants.vroomstickPort);

  // regular drive speed
  private final double speed = 0.85;

  private final JoystickButton outtakeButton = new JoystickButton(vroomstick, 5);
  private final JoystickButton intakeButton = new JoystickButton(vroomstick, 6);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    double FORWARDS = -1.0;
    double BACKWARDS = 1.0;
    double SPEED = 0.7;
    double TURNSPEED = 0.5;
    double LEFT = -1.0;
    double RIGHT = 1.0;
    double TIME_SCALE = 1.0;
    double STRAIGHT = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Auto Chooser in ShuffelBoard {
     // Default Auto (when you turn of the robot, it will change auto back to this)
    autoChooser.setDefaultOption("two Cube Score Red", twoCubeScore_Red(intake, drive));
     //adds options in drop down in the shuffleboard
    autoChooser.addOption("two Cube Score Blue", twoCubeScore_Blue(intake, drive));
    // Places a dropdown in the shuffleboard NOT in the SmartBoard.
    SmartDashboard.putData("Auto modes", autoChooser);
    // }

    // Clear sticky faults, AKA the yellow flashing on the PDP
    powerDistribution.clearStickyFaults();

    // Sets the drive's default command to "teleop driving"

    drive.setDefaultCommand(Commands.run(
        () -> drive.customDrive(m_driverController.getRawAxis(2) * speed, m_driverController.getRawAxis(1) * speed),
        drive));

    // sets the intake's default command to stop running.
    intake.setDefaultCommand(Commands.run(() -> intake.neutral(), intake));
  }

  private Command twoCubeScore_Blue(Intake intake, Drive drive){

    // Score two cube on blue
    Command twoCubeScore_Blue = new SequentialCommandGroup(
        // START SPIT
        Commands.run(() -> intake.backward(), intake).withTimeout(0.5 * TIME_SCALE),
        Commands.run(() -> intake.neutral(), intake).withTimeout(0.2 * TIME_SCALE),

        // MOVE STRAIGHT
        Commands.run(() -> drive.customDrive(0, BACKWARDS * 0.4), drive).withTimeout(.25 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(RIGHT * 0.13, BACKWARDS * SPEED), drive).withTimeout(1.75 * TIME_SCALE),

        // SWERVE
        Commands.run(() -> drive.customDrive(LEFT * .7, BACKWARDS * TURNSPEED)).withTimeout(0.78 * TIME_SCALE),

        // INTAKE ON
        new InstantCommand(() -> intake.forward(), intake),

        // GO TO BALL
        Commands.run(() -> drive.customDrive(0, FORWARDS * SPEED), drive).withTimeout(0.65 * TIME_SCALE),
        Commands.run(() -> intake.forward(), intake).withTimeout(0.3 * TIME_SCALE),

        //stop intake
        new InstantCommand(() -> intake.neutral(), intake),

        // REVERSE
        Commands.run(() -> drive.customDrive(0, BACKWARDS * SPEED), drive).withTimeout(.6 * TIME_SCALE),

        Commands.run(() -> drive.customDrive(LEFT * .6, STRAIGHT), drive).withTimeout(.6 * TIME_SCALE),

        // Drive To Goal
        Commands.run(() -> drive.customDrive(LEFT * 0.25, FORWARDS * SPEED), drive).withTimeout(1.40 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(0, FORWARDS * SPEED), drive).withTimeout(0.88 * TIME_SCALE),
        
        // Spit

        Commands.run(() -> intake.backward(), intake).withTimeout(0.5 * TIME_SCALE),
        Commands.run(() -> intake.neutral(), intake)
    );
    return twoCubeScore_Blue;
  }
  
  private Command twoCubeScore_Red(Intake intake, Drive drive) {

    Command twoCubeScore_Red = new SequentialCommandGroup(
        // START SPIT
        Commands.run(() -> intake.backward(), intake).withTimeout(0.8 * TIME_SCALE),
        Commands.run(() -> intake.neutral(), intake).withTimeout(0.1 * TIME_SCALE),

        // MOVE STRAIGHT
        Commands.run(() -> drive.customDrive(0, BACKWARDS * 0.4), drive).withTimeout(.25 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(0, BACKWARDS * SPEED), drive).withTimeout(1.4 * TIME_SCALE),

        // SWERVE
        Commands.run(() -> drive.customDrive(LEFT * .8, BACKWARDS * TURNSPEED)).withTimeout(0.55 * TIME_SCALE),
        // INTAKE ON
        new InstantCommand(() -> intake.forward(), intake),

        // GO TO BALL
        Commands.run(() -> drive.customDrive(0, FORWARDS * SPEED), drive).withTimeout(1.03 * TIME_SCALE),
        new InstantCommand(() -> intake.neutral(), intake),

        // REVERSE
        Commands.run(() -> drive.customDrive(0, BACKWARDS * SPEED), drive).withTimeout(1 * TIME_SCALE),

        // STRAIGHEN
        Commands.run(() -> drive.customDrive(RIGHT * .8, BACKWARDS * TURNSPEED)).withTimeout(0.55 * TIME_SCALE),

        // DRIVE TO GOAL
        Commands.run(() -> drive.customDrive(0, FORWARDS * SPEED), drive).withTimeout(1.20 * TIME_SCALE),

        // drive to second goal
        Commands.run(() -> drive.customDrive(RIGHT * 0.4, 0), drive).withTimeout(0.4 *TIME_SCALE),
        Commands.run(() -> drive.customDrive(0, FORWARDS * 0.5), drive).withTimeout(0.81 * TIME_SCALE),

        // rmeber to deploy - jo jo

        // SPIT
        Commands.run(() -> intake.backward(), intake).withTimeout(0.5 * TIME_SCALE),
        Commands.run(() -> intake.neutral(), intake)

    // MOVE TO CLIMB
    // Commands.run(() -> drive.customDrive(0, BACKWARDS * SPEED),
    // drive).withTimeout(1 * TIME_SCALE)
    );

    return twoCubeScore_Red; // testing
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.button(5).whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //m_driverController.button(6).whileTrue(new DriveForward(drive));
    m_driverController.button(5).whileTrue(Commands.run(() -> intake.forward(), intake));

    //EventLoop el = new EventLoop();
   // el.bind(() -> Commands.run(() -> intake.forward(), intake));
    //el.bind(() -> System.out.println("Joystick Press!"));
    //vroomstick.button(5, el);
    // TEMP Outtake of right trigger
    m_driverController.button(6).whileTrue(Commands.run(() -> intake.backward(), intake));
  //   m_driverController.axisGreaterThan(1, 0.2).whileTrue(Commands.run(() ->
  //   drive.customDrive(m_driverController.getRawAxis(1), m_driverController.getRawAxis(2)), drive
  // ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    Command sequence1 = new SequentialCommandGroup(
        Commands.run(() -> intake.backward(), intake).withTimeout(0.5 * TIME_SCALE),
        // Commands.run(() -> intake.neutral(), intake),

        Commands.run(() -> drive.customDrive(STRAIGHT, BACKWARDS * SPEED), drive).withTimeout(3.87 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(RIGHT * 0.4, BACKWARDS * SPEED), drive).withTimeout(2.97 * TIME_SCALE),
        Commands.run(() -> intake.forward(), intake), // TODO: double check this
        Commands.run(() -> drive.customDrive(STRAIGHT, FORWARDS * SPEED), drive).withTimeout(1.6 * TIME_SCALE),
        Commands.run(() -> intake.neutral(), intake),
        Commands.run(() -> drive.customDrive(STRAIGHT, BACKWARDS * SPEED), drive).withTimeout(0.9 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(RIGHT * 0.4, FORWARDS * SPEED), drive).withTimeout(2.97 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(STRAIGHT, FORWARDS * SPEED), drive).withTimeout(3.87 * TIME_SCALE),
        Commands.run(() -> intake.backward(), intake).withTimeout(0.5 * TIME_SCALE),
        Commands.run(() -> drive.customDrive(STRAIGHT, BACKWARDS * SPEED), intake).withTimeout(3.87 * TIME_SCALE)
        );

    

    Command testSequenceStr8 = new SequentialCommandGroup(
        Commands.run(() -> drive.customDrive(0, BACKWARDS * 0.8), drive).withTimeout(2));// drive straight
    // 0.85 - 12 feet
    // 0.85 front to front(1 second)- 9ft 8in
    // 0.7 - 6 feet
    // 0.4 front to front(2 second)-
    // 0.4 - 2 feet

    Command testSequenceTurn = new SequentialCommandGroup(
        Commands.run(() -> drive.customDrive(RIGHT * 0.7, FORWARDS * 0.7), drive).withTimeout(1)); // sweep turn
    // 0.4+0.4rotation - 45 degrees/1 second
    // 0.4+0.85rotation - 2.5 full turns with negligible forward movement
    // 0.7+0.7rotation - 170 degree u-turn

    Command testSequenceTurnSharp = new SequentialCommandGroup(
        Commands.run(() -> drive.customDrive(RIGHT * 0.575, 0), drive).withTimeout(1)); // pivot turn
    // 0.4 - just over 60 degrees
    // 0.7 - 270 degrees
    // 0.85 - a little over 1.25 full turns/470-ish degrees

    Command loopbackTest = new SequentialCommandGroup(
        Commands.run(() -> drive.customDrive(0, FORWARDS * 0.7), drive).withTimeout(1),
        Commands.run(() -> drive.customDrive(RIGHT * 0.575, 0), drive).withTimeout(1),
        Commands.run(() -> drive.customDrive(0, FORWARDS * 0.7), drive).withTimeout(1),
        Commands.run(() -> drive.customDrive(RIGHT * 0.575, 0), drive).withTimeout(1));

    return autoChooser.getSelected();
  }

  // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Basic", new
  // PathConstraints(3, 3));
  // HashMap<String, Command> eventMap = new HashMap<>();
  // eventMap.put("IntakeOn", Commands.run(() -> System.out.println("IntakeOn")));
  // eventMap.put("IntakeOff", Commands.run(() ->
  // System.out.println("IntakeOff")));

  // return new SequentialCommandGroup(new InstantCommand(()-> {
  // drive.resetOdometry(examplePath.getInitialPose());
  // }),
  // new PPMecanumControllerCommand(
  // examplePath,
  // drive::getPose,
  // drive.mecanumDriveKinematics,
  // new PIDController(0,0,0),
  // new PIDController(0,0,0),
  // new PIDController(0,0,0),
  // 1.5,
  // drive::getMechanumWheelSpeeds,
  // true,
  // drive
  // )
  // );
  // RamseteController rController = new RamseteController();
  // RamseteAutoBuilder ramseteCommand =
  // new RamseteAutoBuilder( drive::getPose,
  // drive::resetOdometry,
  // rController,
  // drive.kDriveKinematics,
  // drive::tankDriveVolts,
  // eventMap,
  // drive);
  // return ramseteCommand.fullAuto(examplePath); //
  // Autos.exampleAuto(m_exampleSubsystem);
  // }

  public void teleopInit() {

  }
}
