package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm implements Subsystem {
    private Servo leftArm, rightArm;
    private Servo wrist;

    public Arm (HardwareMap hardwareMap){
        rightArm = hardwareMap.servo.get(RobotConstants.Arm.rightArm);
        leftArm = hardwareMap.servo.get(RobotConstants.Arm.leftArm);

        wrist = hardwareMap.servo.get(RobotConstants.Arm.wrist);
    }

    public void setPosition(State state){
        switch(state){
            case IDLE:
                setAssembly(RobotConstants.Arm.armIdle, RobotConstants.Arm.wristIdle, 0, false);
                break;
            case SUB_INTAKING:
                setAssembly(RobotConstants.Arm.armSubmersible, RobotConstants.Arm.wristSubmersible, 0, true);
                break;
            case SUB_GRABBING:
                setAssembly(RobotConstants.Arm.armSubmersibleGrab, RobotConstants.Arm.wristSubmerisbleGrab, 0, false);
            case TRANSFER:
                setAssembly(RobotConstants.Arm.armTransfer, RobotConstants.Arm.wristTransfer, 0, false);
                break;
        }
    }

    public void setAssembly(double target, double wristTarget, double rotationTarget, boolean clawOpen){
        setArms(target);
        setWrist(wristTarget);
    }

    public void setWrist(double target){
        wrist.setPosition(target);
    }

    public void setArms(double target) {
        leftArm.setPosition(target);
        rightArm.setPosition(target);
    }
}
