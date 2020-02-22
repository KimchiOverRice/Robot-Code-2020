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
  final CANSparkMax flywheelLeft = new CANSparkMax(Constants.flywheelleft, MotorType.kBrushless);
  final CANSparkMax flywheelRight = new CANSparkMax(Constants.flywheelRight, MotorType.kBrushless);
  final Compressor compressor = new Compressor(Constants.compressor);
  final DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.leftSolenoidP1,Constants.leftSolenoidP2);
  final DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.rightSolenoidP1,Constants.rightSolenoidP2);
  
  public enum HoodPosition { UP , DOWN };

  private double[] flywheelVelocities = {200, 170};
  private double flywheelTargetVelocity;
  public final double LOW_HOOD_MAX_VELOCITY = 20;
  
  private CANEncoder flywheelEncoder;
  private CANPIDController flyWheelPIDController;
  NetworkTableEntry rpmDisplay, leftCurrent, rightCurrent, leftVoltage, rightVoltage, velocity, targetVelocity;

  public Shooter() {
    flywheelTargetVelocity = 0;

    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();

    rpmDisplay = Shuffleboard.getTab("Testing").add("Shooter RPM", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rightCurrent = Shuffleboard.getTab("Testing").add("Right Current", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    leftCurrent = Shuffleboard.getTab("Testing").add("Left Current", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rightVoltage = Shuffleboard.getTab("Testing").add("Right Voltage", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    velocity = Shuffleboard.getTab("Testing").add("Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    targetVelocity = Shuffleboard.getTab("Testing").add("Flywheel Target Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

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
    flywheelEncoder = flywheelLeft.getEncoder();
    flywheelRight.follow(flywheelLeft, true);
    //Shuffleboard.getTab("Testing").add("Velocity", encoder);

  }

  public double getTargetFlywheelVelocity(){
    return flywheelTargetVelocity;
  }

  public void setTargetFlywheelVelocity(int index){
    flywheelTargetVelocity = flywheelVelocities[index];
    System.out.print(index);
  }

  public double getCurrentShooterVelocity(){
    //System.out.println(encoder.getVelocity());
    return flywheelEncoder.getVelocity();
  }

  public void setSpeed(double speed)
  {
    //System.out.println(leftWheel.getOutputCurrent());
    flywheelLeft.set(speed);
    //rightWheel.set(-speed);
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

  public void flywheelPIDToTargetVelocity() {
    flyWheelPIDController.setReference(flywheelTargetVelocity, ControlType.kVelocity);
    
  }

  public void goToTargetHoodPosition(){
    if(getTargetFlywheelVelocity() > LOW_HOOD_MAX_VELOCITY)
      hoodUp();
    else 
      hoodDown();
  }

  public void stopFlywheel(){
    flywheelTargetVelocity = 0;
  }

  public boolean flywheelAtTargetVelocity(){
    return Math.abs(getTargetFlywheelVelocity()-getCurrentShooterVelocity())<1;
  }


  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    rpmDisplay.setDouble(flywheelEncoder.getVelocity());
    targetVelocity.setDouble(flywheelTargetVelocity);
    //compressor.start();
  }

}
