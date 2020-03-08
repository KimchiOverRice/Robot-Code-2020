/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Cerealizer.Mode;

public class IntakeBall extends CommandBase {

  private Intake intake;
  private Cerealizer cerealizer;
  /**
   * Creates a new intakeBall.
   */

  public IntakeBall(Intake intake, Cerealizer cerealizer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(cerealizer);
    this.intake = intake;
    this.cerealizer = cerealizer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRollerSpeed(1);
    cerealizer.setSpeedCerealizer(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerSpeed(0);
    cerealizer.setSpeedCerealizer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
