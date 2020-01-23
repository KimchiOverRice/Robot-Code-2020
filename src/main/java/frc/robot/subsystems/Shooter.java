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

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  final CANSparkMax leftWheel = new CANSparkMax(6, MotorType.kBrushless);
  final CANSparkMax rightWheel = new CANSparkMax(12, MotorType.kBrushless);
  final Compressor squeezer = new Compressor(0);
  final DoubleSolenoid leftSolenoid = new DoubleSolenoid(1,2);
  final DoubleSolenoid rightSolenoid = new DoubleSolenoid(0,3);
  private CANEncoder encoder;

  public Shooter() {
    //testMotor.enableVoltageCompensation(12);
    encoder = leftWheel.getEncoder();
    //Shuffleboard.getTab("Testing").add("Velocity", encoder);
    rightWheel.follow(leftWheel);
  }

  public double getVelocity(){
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

  public void leftClosed() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  //public void rightClosed
}
