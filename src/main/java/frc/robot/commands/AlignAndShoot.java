/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Limelight;
import frc.robot.Limelight.LightMode;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AlignAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new AlignAndShoot.
   */
  Cerealizer cerealizer;
  DriveTrain driveTrain;
  Compressor compressor;
  Shooter shooter;

  public AlignAndShoot(Cerealizer cerealizer, DriveTrain driveTrain, Shooter shooter, Compressor compressor ) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new EnterShooterMode(cerealizer, shooter, driveTrain, compressor), new ShootBall(shooter, cerealizer));
  }

  public void end(boolean interrupted){
    super.end(interrupted);
    shooter.stopFlywheel();
    driveTrain.stopDrivetrain();
    compressor.start();
    Limelight.setLedMode(LightMode.eOff);
  }
}
