package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class VoltagePID {
        public static double TARGET_VOLTAGE = 14;

        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 0.0;
    }

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

        public static double loweringP = 0.0001;
        public static double loweringI = 0.2;
        public static double loweringD = 0.0006;
        public static double loweringF = 0.00001;

        public static double P = 0.002;
        public static double I = 0.2;
        public static double D = 0.0008;
        public static double F = 0.00001;

        // POSITIONS
        public static int idle = 0;
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
        public static String rotation = "rotation";
        public static String claw = "claw";

        public static double P = 1;
        public static double I = 0.00001;
        public static double D = 0.00001;
        public static double F = 0.00001;

        public static double clawOpen = 0.2;
        public static double clawClose = 0.55;

        public static double armIdle = 0.5;
        public static double wristIdle = 0.75;
        public static double rotationIdle = 0.5;

        public static double armTransfer = 0;
        public static double wristTransfer = 0;

        public static double armSubmersible = 0.55;
        public static double wristSubmersible = 0;
        public static double submerisbleDelay = 0.15;
        public static double armSubmersibleGrab = 0.65;
        public static double wristSubmerisbleGrab = 0.1;
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
