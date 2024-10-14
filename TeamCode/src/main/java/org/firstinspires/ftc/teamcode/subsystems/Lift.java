package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.commands.LiftPID;
import org.firstinspires.ftc.teamcode.commands.RobotConstants;

public class Lift implements Subsystem {

    private DcMotorEx leftSlide, rightSlide;

    private LiftPID leftPID, rightPID;
    int target;

    public Lift(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPID = RobotConstants.Lift.leftPID;
        rightPID = RobotConstants.Lift.rightPID;

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPosition(State state){
        switch(state){
            case SUBINTAKING:
                setTarget(RobotConstants.Lift.Submerisble);
                break;
        }
    }

    public void setTarget(int position) {
        target = position;

        leftPID.clearError();
        rightPID.clearError();
        leftPID.setTarget(target);
        rightPID.setTarget(target);
    }
}
