package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

public class Poses {

    @Config
    public static class blueBucket {
        public static Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
    }
}
