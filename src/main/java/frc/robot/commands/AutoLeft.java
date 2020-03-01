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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoLeft extends SequentialCommandGroup {
  /**
   * Creates a new LeftAuto.
   */
  DriveTrain driveTrain;
  Cerealizer cerealizer;
  Shooter shooter;
  Intake intake;

  public AutoLeft(DriveTrain driveTrain, Shooter shooter, Cerealizer cerealizer, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.cerealizer = cerealizer;
    this.intake = intake;
    
    addRequirements(driveTrain);
    addRequirements(shooter);
    addRequirements(cerealizer);
    addRequirements(intake);
    String trajectoryJSON = "paths/leftauto.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory trajectory;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      addCommands(new InstantCommand(()-> intake.intakeDown()), parallel(new RamseteCommand(trajectory, driveTrain::getRobotPosition,new RamseteController(),
      new SimpleMotorFeedforward(0.22,1.98,0.2),driveTrain.getKinematics(),driveTrain::getWheelSpeeds,
      new PIDController(0.01, 0, 0),
      new PIDController(0.01, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrain::setVoltage,driveTrain), new IntakeBall(intake,cerealizer)));
    addCommands(new AlignShooter(shooter, driveTrain), new ShootBall(shooter, cerealizer));

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
