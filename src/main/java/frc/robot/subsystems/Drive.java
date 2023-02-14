// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  private static CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
  private static CANSparkMax motor3 = new CANSparkMax(3, MotorType.kBrushless);
  private static CANSparkMax motor4 = new CANSparkMax(4, MotorType.kBrushless);

  private static MotorControllerGroup rightGroup = new MotorControllerGroup(motor1, motor2);
  private static MotorControllerGroup leftGroup = new MotorControllerGroup(motor3, motor4);

  private static DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
  
  public void Drive() {
    // leftGroup.setInverted(true);
    
    // rightGroup.setInverted(true);
    //differentialDrive.setSafetyEnabled(false);
    // this.setDefaultCommand(Commands.run(
    //   () ->
    //   differentialDrive.arcadeDrive(0, 0),
    //   this
    // ));
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
