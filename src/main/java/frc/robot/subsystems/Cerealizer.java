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
  final DigitalInput breakBeamShooter = new DigitalInput(Constants.breakBeamOut);
  final DigitalInput breakBeamJam = new DigitalInput(Constants.breakBeamJam);

  final DigitalInput positionZeroLimit = new DigitalInput(Constants.positionZero);
  private int holeNumber, numSpins;
  public static final double DISTANCE_BETWEEN_HOLES = 10;

  public enum Mode {
    INTAKE, SHOOTER
  };

  NetworkTableEntry breakSensorDisplay, holeFullSensor;
  NetworkTableEntry[] holeToggles = new NetworkTableEntry[5];

  public Cerealizer() {
    holeNumber = 0;
    numSpins = 0;

    rotationEncoder = cerealMotor.getEncoder();
    pidController = cerealMotor.getPIDController();
    setEncoderPosition(0);
    pidController.setP(0.01);
    pidController.setI(1e-6);
    pidController.setD(0);

    cerealMotor.burnFlash();    
    cerealMotor.setOpenLoopRampRate(1);
    cerealMotor.setClosedLoopRampRate(1);

    breakSensorDisplay = Shuffleboard.getTab("Testing").add("break beam sensor 1", false)
        .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    holeFullSensor = Shuffleboard.getTab("Testing").add("Hole Full?", false).withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    for (int a = 0; a < holeToggles.length; a++){
      holeToggles[a] = Shuffleboard.getTab("Testing").add("hole " + a, false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
  }

  public boolean atPositionZero() {
    return positionZeroLimit.get();
  }

  public void setEncoderPosition(double rotationPosition) {
    rotationEncoder.setPosition(rotationPosition);
  }

  public void stopCerealMotor() {
    cerealMotor.stopMotor();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  // CURRENTLY CONTROLLED BY SHUFFLEBOARD (for testing purposes)
  public boolean intakeHoleFull() {
    return holeFullSensor.getBoolean(false);
    // return breakBeamBallInside.get();
  }

  public boolean shooterHoleEmpty() {
    return breakBeamShooter.get();
  }

  public void setRotations(double rotations) {
    pidController.setReference(rotations, ControlType.kPosition);
  }

  public void incrementHoleNumber() {
    holeNumber++;
  }

  public void incrementNumSpins() {
    numSpins++;
  }

  public int getCurrentHole() {
    return holeNumber % 5;
  }

  public int getNextHole() {
    return (holeNumber + 1) % 5;
  }

  public double getHolePosition(int hole, Mode mode) {
    return (hole + numSpins * 5) * DISTANCE_BETWEEN_HOLES + (mode == Mode.INTAKE ? 0 : DISTANCE_BETWEEN_HOLES / 2);
  }

  public void trackHoles(Mode mode) {
    holesFilled[getCurrentHole()] = (mode == Mode.INTAKE ? intakeHoleFull() : !shooterHoleEmpty());
  }

  public void setSpeedCerealizer(double speed) {
    cerealMotor.set(speed);
  }

  public void setHoleFull() {
    holesFilled[getCurrentHole()] = true;
  }

  public double getNearestTargetHole(Mode mode) {
    double currentPosition = getRotationPosition();
    double closestTargetPosition;
    double targetPosition = currentPosition;
    double minDist = 50;
    double distBtwHoles = 0;
    for (int holeIndex = 0; holeIndex < holesFilled.length; holeIndex++) {
      if (holesFilled[holeIndex] == (mode == Mode.SHOOTER)) {
        closestTargetPosition = closestEncoderPosition(currentPosition, holeIndex, mode);
        distBtwHoles = Math.abs(currentPosition - closestTargetPosition);
        System.out.println(closestTargetPosition);
        if (distBtwHoles < minDist) {
          targetPosition = closestTargetPosition;
          minDist = distBtwHoles;
          
        }

      }

    }
    return targetPosition;
  }

  public double closestEncoderPosition(double currentPos, double target, Mode mode) {
    double offset = target/5 + (mode == Mode.INTAKE ? 0 : 0.5);
    double position = Math.round((currentPos / (5 * DISTANCE_BETWEEN_HOLES)) - offset);
    double targetRotation = (position + offset) * (5* DISTANCE_BETWEEN_HOLES);
    
    return targetRotation;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spins", numSpins);
    breakSensorDisplay.setBoolean(intakeHoleFull());

    for(int a = 0; a < holeToggles.length; a++){
      holesFilled[a] = holeToggles[a].getBoolean(false);
      SmartDashboard.putBoolean("holes filled " + a, holesFilled[a]);
    }

    SmartDashboard.putNumber("Cereal Pos", rotationEncoder.getPosition());
    SmartDashboard.putNumber("spin Velocity", rotationEncoder.getVelocity());
  }
}
