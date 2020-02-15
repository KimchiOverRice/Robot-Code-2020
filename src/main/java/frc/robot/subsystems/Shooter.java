/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class Shooter extends SubsystemBase {
  final CANSparkMax flywheelLeft = new CANSparkMax(Constants.leftWheels, MotorType.kBrushless);
  final CANSparkMax flywheelRight = new CANSparkMax(Constants.rightWheels, MotorType.kBrushless);
  final Compressor compressor = new Compressor(Constants.compressor);
  final DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.leftSolenoidP1,Constants.leftSolenoidP2);
  final DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.rightSolenoidP1,Constants.rightSolenoidP2);

  private static final double height = 0;
  private static final double kMountAngle = 0;
  private double targetHeight = 2.5; //98.5/12
  
  private CANEncoder encoder;
  private CANPIDController flyWheelPIDController;
  NetworkTableEntry rpmDisplay, leftCurrent, rightCurrent, leftVoltage, rightVoltage, velocity;

  public Shooter() {
    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();

    rpmDisplay = Shuffleboard.getTab("Testing").add("Shooter RPM", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rightCurrent = Shuffleboard.getTab("Testing").add("Right Current", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    leftCurrent = Shuffleboard.getTab("Testing").add("Left Current", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rightVoltage = Shuffleboard.getTab("Testing").add("Right Voltage", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    velocity = Shuffleboard.getTab("Testing").add("Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    flyWheelPIDController = flywheelLeft.getPIDController();
    flyWheelPIDController.setP(0.0001);
    flyWheelPIDController.setI(0.00000001);
    flyWheelPIDController.setD(0.001);

    flywheelLeft.setSmartCurrentLimit(35);
    flywheelRight.setSmartCurrentLimit(35);

    flywheelLeft.setOpenLoopRampRate(1);
    flywheelLeft.setClosedLoopRampRate(1);

    flywheelRight.setOpenLoopRampRate(1);
    flywheelRight.setClosedLoopRampRate(1);


    //testMotor.enableVoltageCompensation(12);
    encoder = flywheelLeft.getEncoder();
    flywheelRight.follow(flywheelLeft, true);
    //Shuffleboard.getTab("Testing").add("Velocity", encoder);
  }

  public double getVelocity(){
    //System.out.println(encoder.getVelocity());
    return encoder.getVelocity();
  }

  public void setSpeed(double speed)
  {
    //System.out.println(leftWheel.getOutputCurrent());
    flywheelLeft.set(speed);
    //rightWheel.set(-speed);
  }


  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    rpmDisplay.setDouble(encoder.getVelocity());
    compressor.start();

    setVelocity(velocity.getDouble(0));
  }

  public void hoodDown() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodUp(){
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void compress(){
    compressor.start();
  }

  public void setVelocity(double velocity) {
    flyWheelPIDController.setReference(velocity, ControlType.kVelocity);
  }
  public double getDistToTarget(){
		return (targetHeight - height) /Math.tan(Math.toRadians(Limelight.getTy() + kMountAngle));
  }

}
