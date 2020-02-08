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

  final CANSparkMax inAndOut = new CANSparkMax(Constants.ejectBall, MotorType.kBrushless);
  final CANSparkMax cerealMotor1 = new CANSparkMax(Constants.spin1, MotorType.kBrushless);
  final CANSparkMax cerealMotor2 = new CANSparkMax(Constants.spin2, MotorType.kBrushless);
  final CANEncoder rotationEncoder;
  private CANPIDController pidController;

  final DigitalInput breakBeamBallInside = new DigitalInput(Constants.breakBeamBallInside);
  final DigitalInput breakBeamShooter = new DigitalInput(Constants.breakBeamOut);
  final DigitalInput breakBeamJam = new DigitalInput(Constants.breakBeamJam);

  final DigitalInput positionZeroLimit = new DigitalInput(Constants.positionZero);
  public static final double DISTANCE_BETWEEN_HOLES = 10;
  public static final double FULL_ROTATION = DISTANCE_BETWEEN_HOLES * 5;

  public enum Mode {
    INTAKE, SHOOTER
  };

  NetworkTableEntry breakSensorDisplay, holeFullSensor;
  NetworkTableEntry[] holeToggles = new NetworkTableEntry[5];

  public Cerealizer() {

    rotationEncoder = cerealMotor1.getEncoder();
    pidController = cerealMotor1.getPIDController();
    setEncoderPosition(0);
    pidController.setP(0.01);
    pidController.setI(1e-6);
    pidController.setD(0);

    cerealMotor1.burnFlash();
    cerealMotor1.setOpenLoopRampRate(1);
    cerealMotor1.setClosedLoopRampRate(1);
    cerealMotor2.follow(cerealMotor1);
    

    breakSensorDisplay = Shuffleboard.getTab("Testing").add("break beam sensor 1", false)
        .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    holeFullSensor = Shuffleboard.getTab("Testing").add("Hole Full?", false).withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    for (int a = 0; a < holeToggles.length; a++) {
      holeToggles[a] = Shuffleboard.getTab("Testing").add("hole " + a, false).withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();
    }
  }

  public boolean atPositionZero() {
    return positionZeroLimit.get();
  }

  public void setEncoderPosition(double rotationPosition) {
    rotationEncoder.setPosition(rotationPosition);
  }

  public void stopCerealMotor() {
    cerealMotor1.stopMotor();
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

  public int getCurrentHole(Mode mode) {
    double rotationPos = rotationEncoder.getPosition();
    if (mode == Mode.INTAKE && Math.round(rotationPos % DISTANCE_BETWEEN_HOLES) == 0) {
      return (int) (Math.round((rotationPos % (FULL_ROTATION)) / DISTANCE_BETWEEN_HOLES));
    } else if (mode == Mode.SHOOTER && Math.round(rotationPos % DISTANCE_BETWEEN_HOLES) == DISTANCE_BETWEEN_HOLES / 2) {
      return (int) (Math.round(((rotationPos + FULL_ROTATION / 2) % FULL_ROTATION) / DISTANCE_BETWEEN_HOLES));
    }
    return -1;
  }

  public void trackHoles(Mode mode) {
    holesFilled[getCurrentHole(mode)] = (mode == Mode.INTAKE ? intakeHoleFull() : !shooterHoleEmpty());
  }

  public void setSpeedCerealizer(double speed) {
    cerealMotor1.set(speed);
  }

  public void setCurrentHoleFull() {
    if(getCurrentHole(Mode.INTAKE) != -1)
      holesFilled[getCurrentHole(Mode.INTAKE)] = true;
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

  public double closestEncoderPosition(double currentPos, int target, Mode mode) {
    double offset = (double) target / 5.0 + (mode == Mode.INTAKE ? 0 : 0.5);
    double position = Math.round((currentPos / (5 * DISTANCE_BETWEEN_HOLES)) - offset);
    double targetRotation = (position + offset) * (5 * DISTANCE_BETWEEN_HOLES);

    return targetRotation;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    breakSensorDisplay.setBoolean(intakeHoleFull());

    for (int a = 0; a < holeToggles.length; a++) {
      holesFilled[a] = holeToggles[a].getBoolean(false);
      SmartDashboard.putBoolean("holes filled " + a, holesFilled[a]);
    }
    SmartDashboard.putNumber("hole Number intake", getCurrentHole(Mode.INTAKE));
    SmartDashboard.putNumber("hole Number shooter", getCurrentHole(Mode.SHOOTER));
    SmartDashboard.putNumber("Cereal Pos", rotationEncoder.getPosition());
    SmartDashboard.putNumber("spin Velocity", rotationEncoder.getVelocity());
  }
}
