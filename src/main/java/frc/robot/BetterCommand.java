/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Like a command... but better
 * (this is a workaround for the annoying order in command.start() that makes execute() run once before checking isFinished)
 */
public abstract class BetterCommand extends CommandBase{
    protected abstract void betterExecute();
    public void execute(){
        if(!isFinished()){
            betterExecute();
        }
    }
}
