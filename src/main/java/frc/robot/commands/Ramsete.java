/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

public class Ramsete extends CommandBase {
  /**
   * Creates a new Ramsete.
   */
  String trajectoryJSON;
  Path trajectoryPath;
  Trajectory trajectory;
  DriveTrain drivetrain;
  RamseteCommand ramseteCommand;

  public Ramsete(DriveTrain drivetrain, String trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    trajectoryJSON = trajectory;
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    ramseteCommand = new RamseteCommand(this.trajectory, drivetrain::getRobotPosition, new RamseteController(),
    new SimpleMotorFeedforward(0.22, 1.98, 0.2), drivetrain.getKinematics(), drivetrain::getWheelSpeeds,
    new PIDController(0.01, 0, 0), new PIDController(0.01, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::setVoltage, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ramseteCommand.initialize();

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ramseteCommand.execute();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
