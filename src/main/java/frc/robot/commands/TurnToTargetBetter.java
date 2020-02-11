/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Limelight;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToTargetBetter extends PIDCommand {
  /**
   * Creates a new TurnToTargetBetter.
   */
  private DriveTrain driveTrain;
  public TurnToTargetBetter(DriveTrain driveTrain) {

    super(
    
        // The controller that the command will use
        new PIDController(.035, 0, 0),
        // This should return the measurement
        () -> Limelight.getTx(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          if(output<0){
            output = output - DriveTrain.MIN_POWER;
          }
          else if(output>0){
            output = output + DriveTrain.MIN_POWER; 
          }
          driveTrain.setSpeed(output, -output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
