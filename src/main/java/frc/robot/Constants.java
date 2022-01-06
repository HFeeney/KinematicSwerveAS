// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double METERS_PER_INCH = 0.0254;

    public static class Swerve {

        public static class PIDGains {
            public static final double KP = 1;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
        }

        public static class ModulePositions {
            // translation 2d considers the front of the robot as the positive x direction
            // and the left of the robot as the positive y direction
            public static final Translation2d FRONT_LEFT = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
            public static final Translation2d FRONT_RIGHT = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
            public static final Translation2d BACK_LEFT = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
            public static final Translation2d BACK_RIGHT = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        }

        public static class Ports {
            // the motor ports should be in the order FL, FR, BL, BR
            public static final int[] SPEED_MOTORS = {6, 7, 2, 8};
            public static final int[] ANGLE_MOTORS = {3, 5, 1, 4};
            public static final int[] ANGLE_ENCODERS = {3, 0, 2, 1};
            public static final Port GYRO = SPI.Port.kMXP;
        }

        public static class AngleEncoder {
            // in encoder counts
            // the number that must be added to the setpoint of the module's rotation (one per module)
            // i.e. the value of the absolute encoder when the module is straight
            public static final double[] OFFSETS = {0.963, 2.216, 0.820, 4.733};

            // in encoder counts per revolution
            // CCW from above is positive direction
            public static final double CPR = 4.899;

            /** 
             * 
             * MODULE  CPR MEASUREMENT  OFFSET MEASUREMENT
             *  0       4.817           0.963
             *  1       4.918           2.216
             *  2       4.930           0.820
             *  3       4.929           4.733
             *  AVG     4.8985          N/A
             */
        }

        public static class TalonEncoder {
            // in counts per revolution
            public static final double CPR = 2048;
        }

        public static class GearRatios {
            // These are gear ratios, the number of rotations of driving gear per rotation of driven gear
            public static final double DRIVE = 9.79 / 1;// 4.65 / 1}; // TODO: these need to be updated
            public static final double TURN = 16 / 1;
        }

        // TODO: what is this really?
        public static final Pose2d INITAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

        public static final int NUM_MODULES = 4;
        
        // in meters
        public static final double WHEEL_BASE = 0.54;
        public static final double TRACK_WIDTH = 0.54;
        public static final double WHEEL_DIAMETER = 3.75 * METERS_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // in meters per second
        public static final double MAX_WHEEL_SPEED = 1; // TODO: unofficial number

        // in radians per second
        public static final double MAX_ANGULAR_SPEED = 2; // TODO: unoffical number

        public static final double JOYSTICK_DEADBAND = 0.07; // minimum input to drive method
        
        // find meters per encoder count using 
        // Circumference / Gear Ratio / Counts Per Motor Revolution = 
        // meters per rev wheel / rev motor per rev wheel / counts per rev motor
        public static final double METERS_PER_COUNT =  WHEEL_CIRCUMFERENCE / GearRatios.DRIVE / TalonEncoder.CPR;
    }

    // this class contains port, axis, and button numbers for the logitech f310 controller
    public static class Joysticks {
        public static final int DRIVER_PORT = 0;

        // buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;

        // stick x axes (horizontal) are more positive to the right
        // stick y axes (vertical) are more positive downwards
        public static class LeftStick {
            public static final int X = 0;
            public static final int Y = 1;
        }

        public static class RightStick {
            public static final int X = 4;
            public static final int Y = 5;
        }
    }
}
