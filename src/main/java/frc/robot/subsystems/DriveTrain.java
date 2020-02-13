/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  NetworkTableEntry proportionSlider;
  public static final double DEFAULT_SPEED_CONST = 0.01;
<<<<<<< HEAD
=======
  public static final double MIN_POWER = 0.05;
>>>>>>> 19b374ce9b352d1e7d58b5bf6cd801ab65895153

  public DriveTrain() {
    frontLeft.restoreFactoryDefaults();
    middleLeft.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    middleRight.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    
    frontLeft.setInverted(true);
    frontRight.setInverted(true);

    middleLeft.follow(frontLeft);
    backLeft.follow(frontLeft);
    middleRight.follow(frontRight);
    backRight.follow(frontRight);

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DriveTrain P", getP());
    SmartDashboard.putBoolean("left invert", frontLeft.getInverted());
    SmartDashboard.putBoolean("right invert",frontRight.getInverted());
    // This method will be called once per scheduler run
  }
}
