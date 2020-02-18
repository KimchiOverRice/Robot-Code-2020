/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ApproachTarget extends CommandBase {
  /**
   * Creates a new ShootBall.
   */
  private Shooter shooter;
  private double distanceToTarget;
  private double targetDistance;
  private DriveTrain driveTrain;

  //private NetworkTableEntry limelightDis;
  public ApproachTarget(Shooter shooter, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
    this.driveTrain = driveTrain;
    distanceToTarget = Limelight.getDistToTarget();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroEncoder();
    targetDistance = driveTrain.getNearestTarget(driveTrain.getNearestTargetIndex());
    shooter.setTargetFlywheelVelocity(driveTrain.getNearestTargetIndex());

    shooter.goToTargetHoodPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveToDistance(distanceToTarget-targetDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(Limelight.getDistToTarget() - targetDistance) <= .25){
      return true;
    }
    return false;
  }
  

}
