// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DriveBackwardDistance;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.NavXTurn;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.Sit;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
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
  //private final Vision vision = new Vision();
  private final PowerDistribution powerDistribution = new PowerDistribution(); // PDP / PDB
  private UsbCamera camera;

  // public static Boolean isSlower = false;

  // Controller for the driver
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final Joystick vroomstick = new Joystick(Constants.OperatorConstants.vroomstickPort);
  
  // regular drive speed
  private double regulerSpeed = 0.5;
  private double turnSpeed = 0.8;
  private double boostspeed = .98;
  private double driveSpeed = regulerSpeed;

  //These values control how the robot speed control operates.
  //IMPORTANT!: Only ONE of these values should be 1, and the other should be zero.
  // when enableDriverBoost is 1 and enableControllerSlider = 0 Robot is slow unless boost button pressed
  private double enableDriverBoost = 0; //Robot is slow unless boost button pressed
  private double enableControllerSlider = 1; //Robot speed is controlled by vroomstick slider
   
  //private final JoystickButton outtakeButton = new JoystickButton(vroomstick, 5);
  //private final JoystickButton intakeButton = new JoystickButton(vroomstick, 6);
  

  private static final double ksVolts = 0.26283;
  private static final double kvVoltSecondsPerMeter = 2.8765;
  private static final double kaVoltSecondsSquaredPerMeter = 0.36244;
  private static final double kPDriveVel = 0.0001;

  public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(0.63); // measured 63 cm from middle to middle
  private static final double kMaxSpeedMetersPerSecond = 0.5;
  private static final double kMaxAccelMetersPerSecond = 0.2;
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
   List.of(new Translation2d(1,0), new Translation2d(2,0)),
   new Pose2d(2,2, new Rotation2d(0)),
   config);

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

   ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Basic", new PathConstraints(1, 1));
   HashMap<String, Command> eventMap = new HashMap<>();

   RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drive::getPose, 
    drive::resetOdometry, 
    new RamseteController(), 
    kDriveKinematics, 
    drive::tankDriveVolts, 
    eventMap, 
    drive);
  Command fullAuto = autoBuilder.fullAuto(pathGroup);

  

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    double FORWARDS = 1.0;
    double BACKWARDS = -1.0;
    double SPEED = 0.7;
    double TURNSPEED = 0.5;
    double LEFT = -1.0;
    double RIGHT = 1.0;
    double TIME_SCALE = 1.0;
    double STRAIGHT = 0;

    //Returns a double where control is the percentage distance between min and max
    //IE: (5, 10, 0.5) returns 7.5
    // (0.1, .9, 1) returns .9 while (0.1, .9, 0) returns .1
    private static final double minMaxSlider(double _min, double _max, double control) {
      return _min + ((_max - _min) * control);
    }

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
    autoChooser.addOption("Do NOTHING", null);
    autoChooser.addOption("Drop, Snag Another, Return", snag_and_180(intake, drive));
    autoChooser.addOption("2 cube basic auto - left", basic_two_cube_auto(false));
    autoChooser.addOption("2 cube basic auto - right", basic_two_cube_auto(true));
    autoChooser.addOption("Taxi", Taxi_Auto());
    autoChooser.addOption("DumpAndJumpOnTheBalance", DumpAndJumpOnTheBalance());
    // Places a dropdown in the shuffleboard NOT in the SmartBoard.
    SmartDashboard.putData("Auto modes", autoChooser);
    // }

    // Clear sticky faults, AKA the yellow flashing on the PDP
    powerDistribution.clearStickyFaults();

    // Sets the drive's default command to "teleop driving"
    drive.setDefaultCommand(Commands.run(
         () -> drive.arcadeDrive(-m_driverController.getRawAxis(1) * ((minMaxSlider(0.4, 0.9, (-vroomstick.getRawAxis(3) + 1) *.5) * enableControllerSlider) + (driveSpeed * enableDriverBoost)) , -m_driverController.getRawAxis(2) * turnSpeed),
         drive));

        

    // sets the intake's default command to stop running.
    intake.setDefaultCommand(Commands.run(() -> intake.neutral(), intake));

    camera = CameraServer.startAutomaticCapture();
  }

  private Command DumpAndJumpOnTheBalance() {
    return new SequentialCommandGroup(
      new OuttakeCommand(intake).withTimeout(0.5),
      Commands.run(() -> intake.neutral(), intake).withTimeout(0.2 * TIME_SCALE),
      new DriveBackwardDistance(drive, 0.90, 55), //TODO: refine Distance
      BasicAutoBalance()
    );
  }

  private Command Taxi_Auto() {
    return new SequentialCommandGroup(
      new OuttakeCommand(intake).withTimeout(0.5)
     // Commands.run(() -> intake.neutral(), intake).withTimeout(0.1),
    //  new DriveBackwardDistance(drive, 0.75, 100)
    );
  }

  private Command BasicAutoBalance() {
    return new SequentialCommandGroup(
      new AutoBalanceCommand(drive),
      new Sit(drive).withTimeout(1),
      new AutoBalanceCommand(drive),
      new Sit(drive).withTimeout(1),
      new AutoBalanceCommand(drive)
    );
  }

  private Command basic_two_cube_auto(boolean turning_right) {
	// clockwise / right turns are positive
	// left turns are negative I guess?
	double rotation_direction = turning_right ? 1 : -1;

    return new SequentialCommandGroup(
      Commands.run(() -> intake.backward(), intake).withTimeout(0.3),
      Commands.run(() -> intake.neutral(), intake).withTimeout(0.05),
      new DriveBackwardDistance(drive, 0.8, 160),
      new Sit(drive).withTimeout(0.2),
      
      new NavXTurn(drive, rotation_direction * 170),
      new Sit(drive).withTimeout(0.2),

      new DriveForwardDistance(drive, 0.6,35, intake).setSpeedScaling(true),
      Commands.run(() -> intake.neutral(), intake).withTimeout(0.1), // intake wasn't turning off
      Commands.run(()-> intake.forward(), intake).withTimeout(0.2), // to deploy
      new Sit(drive).withTimeout(0.2)
      //new NavXTurn(drive, -rotation_direction * 170),
      //new Sit(drive).withTimeout(0.2),
      // comment the remaining out, and build up to it
      //new DriveForwardDistance(drive, 0.8, 179)
      //Commands.run(() -> intake.backward(), intake).withTimeout(0.4)
      //Commands.run(() -> intake.neutral(), intake).withTimeout(0.1)
      );
  }

  private Command snag_and_180(Intake intake, Drive drive){
    // Score two cube on blue
    return new SequentialCommandGroup(
        // START SPIT
        Commands.run(() -> intake.backward(), intake).withTimeout(0.4),
        Commands.run(() -> intake.neutral(), intake).withTimeout(0.05), // probably not required?

        //MOVE BACKWARDS
        new DriveBackwardDistance(drive, SPEED, 100),

        //ROTATE
        new NavXTurn(drive, 180),

        //INTAKE AND DRIVE FORWARD
        new DriveForwardDistance(drive, SPEED, 100, intake),

        //ROTATE
        new NavXTurn(drive, 180),

        //MOVE FORWARD
        new DriveBackwardDistance(drive, SPEED, 200),

        // START SPIT
        Commands.run(() -> intake.backward(), intake).withTimeout(0.4),
        Commands.run(() -> intake.neutral(), intake).withTimeout(0.05) // also probably not required?
    );
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

  
  m_driverController.button(7).whileTrue(Commands.run(()-> driveSpeed = boostspeed));
  m_driverController.button(7).whileFalse(Commands.run(()-> driveSpeed = regulerSpeed)); // USING NOB, NOT BUTTON
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.resetOdometry(exampleTrajectory.getInitialPose());
    return autoChooser.getSelected();
  }

  public void teleopInit() {

  }
}
