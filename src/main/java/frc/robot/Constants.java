/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Solenoids{
        
    }

    public final class CAN {

    }

    public final class DIO{

    }
    //drive train motors
    public static int frontLeft = 1;
    public static int frontRight = 4;
    public static int middleLeft = 2;
    public static int middleRight = 5;
    public static int backLeft = 3;
    public static int backRight = 6; 

    //intake
    public static int intakeLeft1 = 4;
    public static int intakeLeft2 = 6;
    public static int intakeRight1 = 5;
    public static int intakeRight2 = 7;
    public static int rollers = 7;


    //shooter
    public static int flywheelleft = 11; //11
    public static int flywheelRight = 12;
    public static int leftSolenoidP1 = 0;
    public static int leftSolenoidP2 = 1;
    public static int rightSolenoidP1 = 2;
    public static int rightSolenoidP2 = 3;
    public static int compressor = 0;

    //cerealizer
    public static int ejectBall = 10; //10
    public static int spin1 = 8;
    public static int spin2 = 9;
    public static int breakBeamBallInside = 0;
    public static int breakBeamBallMiddle =3;
    public static int breakBeamJam = 4;
    public static int breakBeamOut = 1;
    public static int positionZero = 2;
}
