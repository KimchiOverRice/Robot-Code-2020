/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cerealizer;

public class TurnToEmpty extends CommandBase {
  /**
   * Creates a new TurnToEmpty.
   */
  private Cerealizer cerealizer;
  double currentPosition, numOfRotations, targetPosition, newPosition;
     
  public TurnToEmpty(Cerealizer cerealizer) {
    addRequirements(cerealizer);
    this.cerealizer = cerealizer;
    numOfRotations = 1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = cerealizer.getSlotPosition();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPosition = currentPosition + numOfRotations;
    cerealizer.setRotations(targetPosition);
    newPosition = cerealizer.getSlotPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cerealizer.setRotationSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return newPosition == targetPosition;
  }
}
