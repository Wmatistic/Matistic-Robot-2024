package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

public class RobotConstants {

    @Config
    public static class Mecanum {
        public static String leftFront = "FL";
        public static String leftRear = "BL";
        public static String rightRear = "BR";
        public static String rightFront = "FR";
    }

}
