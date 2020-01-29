/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cerealizer;

public class TurnToEmpty extends CommandBase {
  /**
   * Creates a new TurnToEmpty.
   */
  private Cerealizer cerealizer;
  private int[] holePositions = new int[5];
  double  numOfRotations, targetPosition, newPosition;
     
  public TurnToEmpty(Cerealizer cerealizer) {
    addRequirements(cerealizer);
    this.cerealizer = cerealizer;
    numOfRotations = 1;
    holePositions = cerealizer.getPositionArray();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = holePositions[cerealizer.getNextHole()];
    SmartDashboard.putNumber("Cereal Target Pos", targetPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cerealizer.setRotations(targetPosition);
    newPosition = cerealizer.getRotationPosition();
    SmartDashboard.putNumber("Cereal Pos", newPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cerealizer.stopCerealMotor();
    System.out.println("stopped motor");
    cerealizer.incrementHoleNumber();
    System.out.println("incremented num");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean b = newPosition > targetPosition - 1 && newPosition < targetPosition + 1;
    System.out.println("finished: " + b);
    return b;
  }
}
