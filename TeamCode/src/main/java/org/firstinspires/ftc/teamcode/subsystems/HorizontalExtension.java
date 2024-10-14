package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.State;

public class HorizontalExtension implements Subsystem {

    private Servo rail, leftSlide, rightSlide;

    private double target = 0.0;

    public HorizontalExtension(HardwareMap hardwareMap) {
        rail = hardwareMap.servo.get(RobotConstants.HorizontalExtension.rail);
        leftSlide = hardwareMap.servo.get(RobotConstants.HorizontalExtension.leftSlide);
        rightSlide = hardwareMap.servo.get(RobotConstants.HorizontalExtension.rightSlide);
    }

    public void setPosition(State state){
        switch (state){
            case SUB_INTAKING:
                setExtension(RobotConstants.HorizontalExtension.subIntaking);
                break;
        }
    }

    public void setExtension(double target){
        this.target = target;

        if (target * RobotConstants.HorizontalExtension.railMax <= RobotConstants.HorizontalExtension.railMax) {
            rail.setPosition(target);
        } else {
            rail.setPosition(RobotConstants.HorizontalExtension.railMax);
            leftSlide.setPosition((target - 1) * RobotConstants.HorizontalExtension.slideMax);
            rightSlide.setPosition((target - 1) * RobotConstants.HorizontalExtension.slideMax);

        }
    }

    public void incrementExtension(double input){
        setExtension(target + input);
    }

}
