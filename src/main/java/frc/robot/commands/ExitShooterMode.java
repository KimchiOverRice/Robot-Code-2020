/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LightMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ExitShooterMode extends CommandBase {
  /**
   * Creates a new ExitShooterMode.
   */
  private DriveTrain driveTrain;
  private Shooter shooter;
  private Compressor compressor;
  public ExitShooterMode(DriveTrain driveTrain, Shooter shooter, Compressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.compressor = compressor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopFlywheel();
    driveTrain.stopDrivetrain();
    compressor.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
