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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cerealizer extends SubsystemBase {
  boolean[] holesFilled = new boolean[5];

  final CANSparkMax inAndOut = new CANSparkMax(Constants.inAndOut, MotorType.kBrushless);
  final CANSparkMax rotation = new CANSparkMax(Constants.rotation, MotorType.kBrushless);
  final CANEncoder rotationEncoder;
  private CANPIDController pidController;

  final DigitalInput breakBeamBallInside = new DigitalInput(Constants.breakBeamBallInside);
  final DigitalInput breakBeamBallMiddle = new DigitalInput(Constants.breakBeamBallMiddle);
  final DigitalInput breakBeamJam = new DigitalInput(Constants.breakBeamJam);
  final DigitalInput breakBeamOut = new DigitalInput(Constants.breakBeamOut);
  final DigitalInput positionZero = new DigitalInput(Constants.positionZero);

  int slotNum;

  public Cerealizer() {
    rotationEncoder = rotation.getEncoder();
    pidController = rotation.getPIDController();
    pidController.setP(1);
  }

  public void setRotationPosition(double rotationPosition){
    rotationEncoder.setPosition(rotationPosition);
  }

  public void setRotationSpeed(double speed){
    rotation.set(speed);
  }

  public double getSlotPosition(){
    return rotationEncoder.getPosition();
  }

  public boolean getBeamInside(){
    return breakBeamBallInside.get();
  }

  public void setRotations(double rotations){
    pidController.setReference(rotations, ControlType.kPosition);
  }

  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
