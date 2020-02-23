/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.Cerealizer.Mode;

public class SpinCerealizer extends CommandBase {
  /**
   * Creates a new TurnToEmpty.
   */
  private Cerealizer cerealizer;

 

  private Mode mode;

  double numOfRotations, targetPosition, newPosition;
 

  public SpinCerealizer(Cerealizer cerealizer, Mode mode) {
    addRequirements(cerealizer);
    this.cerealizer = cerealizer;
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = cerealizer.getNearestTargetHole(mode);
   System.out.println("running");
   System.out.println(targetPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cerealizer.setRotations(targetPosition);
    newPosition = cerealizer.getRotationPosition();
    //cerealizer.ejectMotorKeepBallIn(); 
    SmartDashboard.putNumber("Cereal Target Pos", targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cerealizer.stopCerealMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(newPosition - targetPosition) <= 0.25) {
      return true;
    }

    return false;
  }

}