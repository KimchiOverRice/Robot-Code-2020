/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LightMode;
import frc.robot.subsystems.DriveTrain;

public class TurnToTarget extends CommandBase {
  /**
   * Creates a new TurnToTarget.
   */
  DriveTrain driveTrain;
  double tx, speedConst;
  NetworkTableEntry proportionSlider, minSpeedSlider;
  static double DEFAULT_SPEED_CONST, DEFAULT_MIN_CONST;


  public TurnToTarget(DriveTrain drive) {
    addRequirements(drive);
    driveTrain = drive;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    proportionSlider = Shuffleboard.getTab("Testing")
    .add("Turning Constant", DEFAULT_SPEED_CONST)
    .withWidget("Number Slider")
    .getEntry();
    minSpeedSlider = Shuffleboard.getTab("Testing")
    .add("Minimum Speed", DEFAULT_MIN_CONST)
    .withWidget("Number Slider")
    .getEntry();

    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    tx = Limelight.getTx();
    speedConst = proportionSlider.getDouble(DEFAULT_SPEED_CONST);
    speed = speedConst*tx;
    if ( tx > 0 ){
      speed = speed + minSpeedSlider.getDouble(DEFAULT_MIN_CONST);
    } else if (tx < 0){
      speed = speed - minSpeedSlider.getDouble(DEFAULT_MIN_CONST);
    }
    
    driveTrain.setSpeed(-speed, speed);
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeed(0,0);
    Limelight.setLedMode(LightMode.eOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tx == 0){
      return true;
    } else {
    return false;
    }
  }
}
