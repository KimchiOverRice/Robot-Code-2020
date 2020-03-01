/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Cerealizer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Cerealizer.Mode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class EnterShooterMode extends ParallelCommandGroup {
  /**
   * Creates a new ToggleShooterMode.
   */
  Shooter shooter;
  public EnterShooterMode(Cerealizer cerealizer, Shooter shooter, DriveTrain driveTrain) {
    super(new AlignShooter(shooter, driveTrain), new SpinCerealizer(cerealizer));
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();

    this.shooter = shooter;
  }

 // public void end(boolean interrupted){
   // super.end(interrupted);
   // if(interrupted)
      //shooter.stopFlywheel();
     // SmartDashboard.putBoolean("interrupted", interrupted);
 // }
}
