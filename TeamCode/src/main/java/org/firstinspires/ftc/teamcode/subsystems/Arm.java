package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm implements Subsystem {
    private Servo leftArm, rightArm;

    public Arm (HardwareMap hardwareMap){
        rightArm = hardwareMap.servo.get(RobotConstants.Arm.rightArm);
        leftArm = hardwareMap.servo.get(RobotConstants.Arm.leftArm);
    }

    public void setPosition(State state){
        switch(state){
            case IDLE:
                setArms(RobotConstants.Arm.idle);
                break;
            case TRANSFER:
                setArms(RobotConstants.Arm.transfer);
                break;
        }
    }

    public void setArms(double target) {
        leftArm.setPosition(target);
        rightArm.setPosition(target);
    }
}
