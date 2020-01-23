/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase {
  /**
   * Creates a new MoveIntake.
   */
  private Intake intake;
  private static double kMinimum = .5;
  private double targetPosition;
  private double offsetPosition;
  double currentPosition;
  public enum Tilt {UP, DOWN};
  
  


  public MoveIntake(Intake intake,Tilt direction){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    if(direction == Tilt.UP){
      targetPosition = 0;
    }
    else if(direction == Tilt.DOWN){
      targetPosition = 200;
    }
    //make if statements for what targetPosition is
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = intake.getTurns();
    offsetPosition = targetPosition - currentPosition;
    if(offsetPosition < 0){
      intake.setArmSpeed((offsetPosition)/400 - kMinimum);
    }else if (offsetPosition > 0){
      intake.setArmSpeed((offsetPosition)/400 + kMinimum);
    }
    SmartDashboard.putNumber("Off set position", offsetPosition);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return offsetPosition <= 10 && offsetPosition >= -10;
  }

}
