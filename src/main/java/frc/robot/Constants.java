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
    //drive train motors
    public static int frontLeft = 1;
    public static int frontRight = 4;
    public static int middleLeft = 2;
    public static int middleRight = 5;
    public static int backLeft = 3;
    public static int backRight = 6;

    //intake
    public static int arm = 7;
    public static int rollers = 8;


    //shooter
    public static int leftWheels = 11;
    public static int rightWheels = 12;
    public static int leftSolenoidP1 = 0;
    public static int leftSolenoidP2 = 1;
    public static int rightSolenoidP1 = 2;
    public static int rightSolenoidP2 = 3;
    public static int compressor = 0;

    //serializer
    public static int inAndOut = 9;
    public static int rotation = 10;
    public static int breakBeamIn = 0;
    public static int breakBeamOut = 1;
    public static int positionZero = 2;
}
