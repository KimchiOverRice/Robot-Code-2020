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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  
  final DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.intakeLeft1, Constants.intakeLeft2);
  final DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.intakeRight1,Constants.intakeRight2);
  final CANSparkMax roller = new CANSparkMax(Constants.rollers, MotorType.kBrushless);
  final DigitalInput breakBeamBallMiddle = new DigitalInput(Constants.breakBeamBallMiddle);
  final DigitalInput breakBeamJam = new DigitalInput(Constants.breakBeamJam);
 

  public Intake() {
    
  }

  public void intakeDown() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void intakeUp(){
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  public void setRollerSpeed(double speed){
    roller.set(speed);
  }

  public boolean fullyIn(){
    return breakBeamBallMiddle.get();
  }
  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
