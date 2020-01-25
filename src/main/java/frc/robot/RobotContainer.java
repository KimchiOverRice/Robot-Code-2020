/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final DriveTrain driveTrain = new DriveTrain();
  final Shooter shooter = new Shooter();
  final Intake intake = new Intake();

  Joystick joystickLeft = new Joystick(1);
  Joystick joystickRight = new Joystick(2);
  JoystickButton joystickbuttonRight = new JoystickButton(joystickRight, 2);
  TurnToTarget turnToTarget = new TurnToTarget(driveTrain);
  private NetworkTableEntry shooterSpeedDisplay;

  private NetworkTableEntry shooterSpeedSlider;

  public double getValueOfLeftY() {
    return joystickLeft.getY();
  }

  public double getValueOfRightY() {
    return joystickRight.getY();
  }

  public double getSpeedFromSlider() {
    return shooterSpeedSlider.getDouble(0.5);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    shooterSpeedSlider = Shuffleboard.getTab("Testing").add("Shooter Speed", 0).withWidget("Number Slider").withPosition(1, 1)
        .withSize(2, 1).getEntry();
    shooterSpeedDisplay = Shuffleboard.getTab("Testing").add("Shooter Speed Display", 0).withWidget(BuiltInWidgets.kTextView).getEntry();  

    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.setSpeed(getValueOfLeftY(), getValueOfRightY()), driveTrain));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setSpeed(getSpeedFromSlider()), shooter));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void setShooterSpeedDisplay(double speed){
    shooterSpeedDisplay.setDouble(speed);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    joystickbuttonRight.whenPressed(turnToTarget);
    new JoystickButton(joystickRight, 5).whenPressed(new InstantCommand(intake::intakeDown, intake));
    new JoystickButton(joystickRight, 3).whenPressed(new InstantCommand(intake::intakeUp, intake)); 
    new JoystickButton(joystickRight, 7).whileHeld(new IntakeBall (intake));
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
