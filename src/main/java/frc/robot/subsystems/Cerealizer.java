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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cerealizer extends SubsystemBase {
  boolean[] holesFilled = new boolean[5];

  final CANSparkMax inAndOut = new CANSparkMax(Constants.inAndOut, MotorType.kBrushless);
  final CANSparkMax cerealMotor = new CANSparkMax(Constants.rotation, MotorType.kBrushless);
  final CANEncoder rotationEncoder;
  private CANPIDController pidController;

  final DigitalInput breakBeamBallInside = new DigitalInput(Constants.breakBeamBallInside);
  final DigitalInput breakBeamBallMiddle = new DigitalInput(Constants.breakBeamBallMiddle);
  final DigitalInput breakBeamJam = new DigitalInput(Constants.breakBeamJam);
  final DigitalInput breakBeamOut = new DigitalInput(Constants.breakBeamOut);
  final DigitalInput positionZero = new DigitalInput(Constants.positionZero);
  private int holeNumber;
  private int[] holePositions = {0,10,20,30,40};

  NetworkTableEntry breakSensorDisplay;
  

  public Cerealizer() {
    holeNumber = 0;
    rotationEncoder = cerealMotor.getEncoder();
    pidController = cerealMotor.getPIDController();
    setEncoderPosition(0);
    pidController.setP(0.01);
    pidController.setI(1e-6);
    pidController.setD(0);
    cerealMotor.burnFlash();
    breakSensorDisplay = Shuffleboard.getTab("Testing").add("break beam sensor 1", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  }

  public void setEncoderPosition(double rotationPosition){
    rotationEncoder.setPosition(rotationPosition);
  }

  public void stopCerealMotor(){
    cerealMotor.stopMotor();
  }

  public double getRotationPosition(){
    return rotationEncoder.getPosition();
  }

  public boolean getBeamInside(){
    return breakBeamBallInside.get();
  }

  public void setRotations(double rotations){
    pidController.setReference(rotations, ControlType.kPosition);
  }

  public void incrementHoleNumber(){
    holeNumber++;
  }

  public int getHoleNumber(){
    return holeNumber%5;
  }

  public int getNextHole(){
    if(holeNumber<4){
      return holeNumber + 1;
    }
    else{
      return 0;
    }
  }

  public int[] getPositionArray(){
    return holePositions;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    breakSensorDisplay.setBoolean(getBeamInside());
  }
}
