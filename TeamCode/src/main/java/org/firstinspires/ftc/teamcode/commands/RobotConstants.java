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

        public static LiftPID leftPID = new LiftPID(3, 0.17, 1, 0, 515);
        public static LiftPID rightPID = new LiftPID(3, 0.17, 1, 0, 515);

        // POSITIONS
        public static int highBucket = 0;
        public static int lowBucket = 0;
        public static int highBar = 0;
        public static int lowBar = 0;
        public static int submersible = 0;
    }

    @Config
    public static class Arm {
        public static String leftArm = "leftArm";
        public static String rightArm = "rightArm";

        public static double idle = 0;
        public static double transfer = 0;
        public static double transferGrab = 0;
        public static double submersible = 0;
        public static double submersibleGrab = 0;
    }

    @Config
    public static class HorizontalExtension {
        public static String rail = "rail";
        public static String leftSlide = "leftSlide";
        public static String rightSlide = "rightSlide";

        public static double railMax = 0.0;
        public static double slideMax = 0.0;

        // Input works on a scale of 2 with 1.0 being full extension of the rail and 2.0 being full extension of both slide and rail
        public static double subIntaking = 0.0;
    }

}
