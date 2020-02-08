/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  final CANSparkMax frontLeft = new CANSparkMax(Constants.frontLeft, MotorType.kBrushless);
  final CANSparkMax frontRight = new CANSparkMax(Constants.frontRight, MotorType.kBrushless);
  final CANSparkMax middleLeft = new CANSparkMax(Constants.middleLeft, MotorType.kBrushless);
  final CANSparkMax middleRight = new CANSparkMax(Constants.middleRight, MotorType.kBrushless);
  final CANSparkMax backLeft = new CANSparkMax(Constants.backLeft, MotorType.kBrushless);
  final CANSparkMax backRight = new CANSparkMax(Constants.backRight, MotorType.kBrushless);

  DifferentialDrive driveTrain;

  public DriveTrain() {
    middleLeft.follow(frontLeft);
    backLeft.follow(frontLeft);
    middleRight.follow(frontRight);
    backRight.follow(frontRight);

    frontLeft.setInverted(true);

    //driveTrain = new DifferentialDrive(frontLeft, frontRight);
    //driveTrain.setDeadband(0.1);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    frontRight.set(rightSpeed);
    frontLeft.set(leftSpeed);
    //driveTrain.tankDrive(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
