package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Works like wpilib ConditionalCommand except it just starts the command & then ends itself, instead of continuing to run until
 * started command is done running.
 */
public class ConditionalStartCommand extends CommandBase{
    Command onTrue, onFalse;
    BooleanSupplier condition;

    public ConditionalStartCommand(Command onTrue, Command onFalse, BooleanSupplier condition){
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
    }
    
    public void initialize(){
        if(condition.getAsBoolean())
            onTrue.schedule();
        else
            onFalse.schedule();

    }

    public boolean isFinished(){
        return true;
    }

}
