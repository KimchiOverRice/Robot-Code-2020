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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  final CANSparkMax testMotor = new CANSparkMax(1, MotorType.kBrushed);

  private CANEncoder encoder;

  public Shooter() {
    //testMotor.enableVoltageCompensation(12);
    encoder = testMotor.getEncoder();
    //Shuffleboard.getTab("Testing").add("Velocity", encoder);
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public void setSpeed(double speed)
  {
    testMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
  }
}
