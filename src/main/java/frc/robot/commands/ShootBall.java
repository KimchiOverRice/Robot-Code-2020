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
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
  /**
   * Creates a new ShootBall.
   */
  private Shooter shooter;

  private static final double height = 0;
  private static final double kMountAngle = 0;
  private double distanceToTarget;
  private double targetHeight = 2.5; //98.5/12
  //private NetworkTableEntry limelightDis;
  public ShootBall(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
    distanceToTarget = getDistToTarget();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Limelight Distance", getDistToTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double getDistToTarget(){
		return (targetHeight - height) /Math.tan(Math.toRadians(Limelight.getTy() + kMountAngle));
	}
}
