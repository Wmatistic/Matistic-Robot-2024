package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Poses {

    @Config
    public static class blueBucket {
        public static Pose2d start = new Pose2d(-16, 61, Math.toRadians(180));
        public static Vector2d blueBucket = new Vector2d(58, 56);
    }
}
