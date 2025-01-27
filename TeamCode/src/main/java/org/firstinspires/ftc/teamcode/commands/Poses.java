package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Poses {

    @Config
    public static class blueBucket {
        public static Pose2d start = new Pose2d(7.5, 61, Math.toRadians(90));
        public static Pose2d highBar = new Pose2d(7.5, 46, Math.toRadians(90));

        public static Pose2d bucket = new Pose2d(58, 56, Math.toRadians(45));
        public static Pose2d firstSample = new Pose2d(47.5, 47.5, Math.toRadians(90));
        public static Pose2d secondSample = new Pose2d(59.5, 50, Math.toRadians(90));
        public static Pose2d thirdSample = new Pose2d(59.5, 48.5, Math.toRadians(112.5));
    }
}
