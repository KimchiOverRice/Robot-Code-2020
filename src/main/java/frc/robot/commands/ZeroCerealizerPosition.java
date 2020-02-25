/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cerealizer;

public class ZeroCerealizerPosition extends CommandBase {
  /**
   * Creates a new ZeroPosition.
   */
  private Cerealizer cerealizer;
  double startTime;
  int timeout;
  Timer timer = new Timer();

  public ZeroCerealizerPosition(Cerealizer cerealizer) {
    // Use addRequirements() here to declare subsystem dependencies.
    

    addRequirements(cerealizer);
    this.cerealizer = cerealizer;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (timer.get()<1){
    cerealizer.setSpeedCerealizer(.1);
  }else{
    cerealizer.setSpeedCerealizer(-.1);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cerealizer.stopCerealMotor();
    if(!interrupted){
    cerealizer.zeroEncoder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cerealizer.atPositionZero();
  }
}
