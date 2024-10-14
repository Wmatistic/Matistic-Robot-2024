package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {

    public Mecanum drivetrain;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Mecanum(hardwareMap);
    }

}
