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

public class TurnToEmpty extends CommandBase {
  /**
   * Creates a new TurnToEmpty.
   */
  private Cerealizer cerealizer;
  double  numOfRotations, targetPosition, newPosition;
  int numFull;
     
  public TurnToEmpty(Cerealizer cerealizer) {
    addRequirements(cerealizer);
    this.cerealizer = cerealizer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = cerealizer.getHolePosition(cerealizer.getNextHole());
    numFull=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cerealizer.setRotations(targetPosition);
    newPosition = cerealizer.getRotationPosition();
    SmartDashboard.putNumber("Cereal Pos", newPosition);
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
    if(Math.abs(newPosition - targetPosition) <= 1){
      cerealizer.stopCerealMotor();
      //TODO: pause it somehow so it actually stops?
      cerealizer.incrementHoleNumber();
      SmartDashboard.putNumber("hole", cerealizer.getCurrentHole());
      if(cerealizer.getCurrentHole() == 4){
        cerealizer.incrementNumSpins();
      }

      if(cerealizer.holeFull()){
        numFull++;
        if(numFull == 4){
          return true;
        }
        targetPosition = cerealizer.getHolePosition(cerealizer.getNextHole());
        return false; 
      }
      return true;
    }
    return false;
  }
}
