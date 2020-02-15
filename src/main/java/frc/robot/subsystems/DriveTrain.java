/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class DriveTrain extends SubsystemBase {
  final CANSparkMax frontLeft = new CANSparkMax(Constants.frontLeft, MotorType.kBrushless);
  final CANSparkMax frontRight = new CANSparkMax(Constants.frontRight, MotorType.kBrushless);
  final CANSparkMax middleLeft = new CANSparkMax(Constants.middleLeft, MotorType.kBrushless);
  final CANSparkMax middleRight = new CANSparkMax(Constants.middleRight, MotorType.kBrushless);
  final CANSparkMax backLeft = new CANSparkMax(Constants.backLeft, MotorType.kBrushless);
  final CANSparkMax backRight = new CANSparkMax(Constants.backRight, MotorType.kBrushless);
  
  
  private CANPIDController driveTrainPIDLeft;
  private CANPIDController driveTrainPIDRight;
  

  AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  DifferentialDrive driveTrain;

  NetworkTableEntry proportionSlider;
  public static final double DEFAULT_SPEED_CONST = 0.01;
  public static final double MIN_POWER = 0;

  public DriveTrain() {
    frontLeft.restoreFactoryDefaults();
    middleLeft.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    middleRight.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();

    middleLeft.follow(frontLeft);
    backLeft.follow(frontLeft);
    middleRight.follow(frontRight);
    backRight.follow(frontRight);

    driveTrainPIDLeft = frontLeft.getPIDController();
    driveTrainPIDLeft.setP(0.0001);
    driveTrainPIDLeft.setI(0.00000001);
    driveTrainPIDLeft.setD(0.001);

    
    driveTrainPIDRight = frontRight.getPIDController();
    driveTrainPIDRight.setP(0.0001);
    driveTrainPIDRight.setI(1e-6);
    driveTrainPIDRight.setD(0.001);

    frontLeft.setOpenLoopRampRate(1);
    frontLeft.setClosedLoopRampRate(1);

    frontRight.setOpenLoopRampRate(1);
    frontRight.setClosedLoopRampRate(1);

    proportionSlider = Shuffleboard.getTab("Testing")
    .add("Turning P",0)
    .withWidget("Number Slider")
    .getEntry();


    driveTrain = new DifferentialDrive(frontLeft, frontRight);
    driveTrain.setDeadband(0.1);

   // PIDController test = new PIDController(0.03,0,0);
    //test.setOutput
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    //frontRight.set(rightSpeed);
    //frontLeft.set(leftSpeed);
    driveTrain.tankDrive(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);


  }

  public double getP(){
    return proportionSlider.getDouble(0);
  }

  public double getTargetAngle(){
    return Limelight.getTx() + gyro.getAngle();
  }

  public double getCurrentAngle(){
    return gyro.getAngle();
  }

  public void zeroGyro(){
    gyro.zeroYaw();
  }

  public void driveToDistance(){
     // driveTrainPIDLeft.setReference(rotations, ControlType.kPosition);
      //driveTrainPIDRight.setReference(rotations, ControlType.kPosition);
      
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getCurrentAngle());
 
    // This method will be called once per scheduler run
  }
}
