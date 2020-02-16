/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HoodPosition;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MoveHood extends InstantCommand {
  private Shooter shooter;
 
  private HoodPosition targetPosition;

  public MoveHood(Shooter shooter, HoodPosition position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
    targetPosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (targetPosition == HoodPosition.UP){
      shooter.hoodUp();
    } else if(targetPosition == HoodPosition.DOWN){
      shooter.hoodDown();
    }
  }
}
