package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class Mecanum {
        public static String leftFront = "FL";
        public static String leftRear = "BL";
        public static String rightRear = "BR";
        public static String rightFront = "FR";
    }

    @Config
    public static class Lift {
        public static String leftSlide = "leftSlide";
        public static String rightSlide = "rightSlide";

        public static LiftPID leftPID = new LiftPID(2, 0.1,4, 0, 515);
        public static LiftPID rightPID = new LiftPID(2, 0.1, 4, 0, 515);

        public static double P = 0.0011;
        public static double I = 0.2;
        public static double D = 0.0001;
        public static double F = 0.00001;

        // POSITIONS
        public static int idle = 10;
        public static int highBucket = 1190;
        public static int lowBucket = 600;
        public static int highBar = 0;
        public static int lowBar = 0;
        public static int submersible = 0;
    }

    @Config
    public static class Arm {
        public static String leftArm = "leftArm";
        public static String rightArm = "rightArm";
        public static String wrist = "wrist";

        public static double armIdle = 1.0;
        public static double wristIdle = 0.75;

        public static double armTransfer = 0;
        public static double wristTransfer = 0;

        public static double armSubmersible = 0.05;
        public static double wristSubmersible = 0.25;
        public static double armSubmersibleGrab = 0;
        public static double wristSubmerisbleGrab = 0.24;
    }

    @Config
    public static class HorizontalExtension {
        public static String rail = "rail";
        public static String leftExtension = "leftExtension";
        public static String rightExtension = "rightExtension";

        public static double railMax = 0.16;
        public static double slideMax = 0.72;

        // Input works on a scale of 2 with 1.0 being full extension of the rail and 2.0 being full extension of both slide and rail
        public static double idle = 0.0;
        public static double subIntaking = 2;
        public static double scoring = 0.0;
    }

}
