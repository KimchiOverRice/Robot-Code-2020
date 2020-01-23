/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  final CANSparkMax arm = new CANSparkMax(12, MotorType.kBrushless);
  final CANSparkMax roller = new CANSparkMax(8, MotorType.kBrushless);
  
  private CANEncoder armEncoder;

  public Intake() {
    armEncoder = arm.getEncoder();
  }

  public void setArmSpeed(double speed){
    arm.set(speed);
  }

  public void setRollerSpeed(double speed){
    roller.set(speed);
  }

  public double getTurns(){
    return armEncoder.getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder",getTurns());
  }
}
