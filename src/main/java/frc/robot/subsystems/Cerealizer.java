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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cerealizer extends SubsystemBase {
  boolean[] holesFilled = new boolean[5];

  final CANSparkMax inAndOut = new CANSparkMax(Constants.inAndOut, MotorType.kBrushless);
  final CANSparkMax rotation = new CANSparkMax(Constants.rotation, MotorType.kBrushless);
  final CANEncoder rotationEncoder;

  final DigitalInput breakBeamIn = new DigitalInput(Constants.breakBeamIn);
  final DigitalInput breakBeamOut = new DigitalInput(Constants.breakBeamOut);
  final DigitalInput positionZero = new DigitalInput(Constants.positionZero);

  public Cerealizer() {
    rotationEncoder = rotation.getEncoder();
  }

  public void setRotationPosition(double rotationPosition){
    rotationEncoder.setPosition(rotationPosition);
  }

  public void setRoationSpeed(double speed){
    rotation.set(speed);
  }

  public double getSlotPosition(){
    return rotationEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
