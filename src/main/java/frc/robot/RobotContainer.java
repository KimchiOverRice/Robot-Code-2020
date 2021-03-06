/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.MoveHood;
import frc.robot.commands.ShootBall;
import frc.robot.Limelight.LightMode;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.ApproachTarget;
import frc.robot.commands.AutoLeft;
import frc.robot.commands.AutoMiddle;
import frc.robot.commands.AutoRight;
import frc.robot.commands.ExitShooterMode;
import frc.robot.commands.SpinCerealizer;
import frc.robot.commands.EnterShooterMode;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Shooter.HoodPosition;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final Compressor compressor = new Compressor(Constants.compressor);
  final DriveTrain driveTrain = new DriveTrain();
  final Shooter shooter = new Shooter();
  final Intake intake = new Intake();
  final Cerealizer cerealizer = new Cerealizer();
  Joystick joystickLeft = new Joystick(1);
  Joystick joystickRight = new Joystick(2);

  Joystick xBox = new Joystick(0);
  Joystick leftStick = new Joystick(1);
  Joystick rightStick = new Joystick(2);

  public JoystickButton aButton = new JoystickButton(xBox, 1);
  public JoystickButton bButton = new JoystickButton(xBox, 2);
  public JoystickButton xButton = new JoystickButton(xBox, 3);
  public JoystickButton yButton = new JoystickButton(xBox, 4);
  public JoystickButton leftBumperButton = new JoystickButton(xBox, 5);
  public JoystickButton rightBumperButton = new JoystickButton(xBox, 6);
  public JoystickButton backButton = new JoystickButton(xBox, 7);
  public JoystickButton startButton = new JoystickButton(xBox, 8);
  public XBoxTrigger leftTrigger = new XBoxTrigger(xBox, 2);
  public XBoxTrigger rightTrigger = new XBoxTrigger(xBox, 3);

  public double leftStickY() {
    return -leftStick.getRawAxis(1);
  }

  public double rightStickY() {
    return -rightStick.getRawAxis(1);
  }

  public double xBoxLeftStickY() {
    return xBox.getRawAxis(1);
  }

  public double xBoxLeftStickX() {
    return xBox.getRawAxis(0);
  }

  public double xBoxRightStickY() {
    return xBox.getRawAxis(5);
  }

  public double xBoxRightStickX() {
    return xBox.getRawAxis(4);
  }

  // JoystickButton joystickbuttonRight = new JoystickButton(joystickRight, 2);
  private NetworkTableEntry shooterSpeedDisplay;

  private NetworkTableEntry shooterSpeedSlider, cerealizerSpeedSlider, intakeRollerSlider;

  public double getValueOfLeftY() {
    return -joystickLeft.getY();
  }

  public void startCompressor(){
    compressor.start();
  }

  public void stopCompressor(){
    compressor.stop();
  }

  public double getValueOfRightY() {
    return -joystickRight.getY();
  }

  public double getSpeedFromSlider() {
    return shooterSpeedSlider.getDouble(0.5);
    
  }

  public double getSpeedForCerealizer() {
    return cerealizerSpeedSlider.getDouble(0);
  }

  public double getSpeedForIntake() {
    return intakeRollerSlider.getDouble(0);
  }

  enum AutoStrategy {
    LEFT, RIGHT, MIDDLE;
  }

  SendableChooser<AutoStrategy> autoStrategy = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    shooterSpeedSlider = Shuffleboard.getTab("Testing").add("Shooter Speed", 0).withWidget("Number Slider")
        .withPosition(1, 1).withSize(2, 1).getEntry();
    shooterSpeedDisplay = Shuffleboard.getTab("Testing").add("Shooter Speed Display", 0)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    cerealizerSpeedSlider = Shuffleboard.getTab("Testing").add("Cerealizer speed", 0).withWidget("Number Slider")
        .withSize(2, 1).getEntry();

    intakeRollerSlider = Shuffleboard.getTab("Testing").add("intake speed", 0).withWidget("Number Slider")
        .withSize(2, 1).getEntry();

    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.setSpeed(getValueOfLeftY(), getValueOfRightY()), driveTrain));

    //cerealizer.setDefaultCommand(new RunCommand(() -> cerealizer.setSpeedCerealizer(0), cerealizer));
    cerealizer.setDefaultCommand(new RunCommand(() -> cerealizer.setSpeedCerealizer(getSpeedForCerealizer()), cerealizer));

    intake.setDefaultCommand(new RunCommand(() -> intake.setRollerSpeed(0), intake));

  //  shooter.setDefaultCommand(new RunCommand(() -> shooter.flywheelPIDToTargetVelocity(), shooter));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setSpeed(getSpeedFromSlider()), shooter));

    

    // Configure the button bindings
    configureButtonBindings();

    autoStrategy.setDefaultOption("Right", AutoStrategy.RIGHT);
    autoStrategy.addOption("Left", AutoStrategy.LEFT);
    autoStrategy.addOption("Middle", AutoStrategy.MIDDLE);
  }

  public void setShooterSpeedDisplay(double speed) {
    shooterSpeedDisplay.setDouble(speed);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // pneumatics
    new JoystickButton(joystickRight, 11).whenPressed(new InstantCommand(() -> compressor.start()));
    new JoystickButton(joystickRight, 10).whenPressed(new InstantCommand(() -> compressor.stop()));

    // shooter
    // new JoystickButton(joystickLeft, 2).whileHeld(new TurnToTarget(driveTrain));
    new JoystickButton(joystickLeft, 12).whenPressed(new MoveHood(shooter, HoodPosition.UP));
    new JoystickButton(joystickLeft, 11).whenPressed(new MoveHood(shooter, HoodPosition.DOWN));

    leftTrigger.whenPressed(new ConditionalStartCommand(new EnterShooterMode(cerealizer, shooter, driveTrain, compressor),
        new ExitShooterMode(driveTrain, shooter, compressor), () -> shooter.getTargetFlywheelVelocity() == 0));

  //  new JoystickButton(joystickRight, 12).toggleWhenPressed(new ShootBall(shooter, cerealizer));

    // intake
    new JoystickButton(joystickLeft, 3).whenPressed(new InstantCommand(intake::intakeDown, intake)); // rightJoystick
    new JoystickButton(joystickLeft, 5).whenPressed(new InstantCommand(intake::intakeUp, intake));

    rightTrigger.whileHeld(new IntakeBall(intake, cerealizer));
    new JoystickButton(joystickLeft, 1).whileHeld(new IntakeBall(intake, cerealizer));
   // new JoystickButton(joystickRight, 8)
       // .whileHeld(new RunCommand(() -> intake.setRollerSpeed(getSpeedForIntake()), intake));
    rightBumperButton
        .whileHeld(new RunCommand(() -> intake.setRollerSpeed(-getSpeedForIntake()), intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    if (autoStrategy.getSelected() == AutoStrategy.RIGHT) {
      return new AutoRight(driveTrain, shooter, cerealizer, intake);
    } else if (autoStrategy.getSelected() == AutoStrategy.LEFT) {
      return new AutoLeft(driveTrain, shooter, cerealizer, intake, compressor);
    } else {
      return new AutoMiddle(driveTrain, shooter, cerealizer, intake);
    }
  }

}
