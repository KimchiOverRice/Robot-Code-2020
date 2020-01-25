/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  final CANSparkMax leftWheel = new CANSparkMax(Constants.leftWheels, MotorType.kBrushless);
  final CANSparkMax rightWheel = new CANSparkMax(Constants.rightWheels, MotorType.kBrushless);
  final Compressor squeezer = new Compressor(Constants.compressor);
  final DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.leftSolenoidP1,Constants.leftSolenoidP2);
  final DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.rightSolenoidP1,Constants.rightSolenoidP2);
  private CANEncoder encoder;

  public Shooter() {
    //testMotor.enableVoltageCompensation(12);
    encoder = leftWheel.getEncoder();
    //Shuffleboard.getTab("Testing").add("Velocity", encoder);
    rightWheel.follow(leftWheel);
    
  }

  public double getVelocity(){
    System.out.println(encoder.getVelocity());
    return encoder.getVelocity();
  }

  public void setSpeed(double speed)
  {
    System.out.println(speed);
    leftWheel.set(speed);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
  }

  public void hoodDown() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodUp(){
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }
}
