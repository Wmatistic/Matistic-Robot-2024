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

    private final DcMotorEx leftSlide, rightSlide;

    private final LiftPID leftPID, rightPID;
    int target;

    public Lift(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.Lift.leftSlide);
        rightSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.Lift.rightSlide);

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
            case IDLE:
                setTarget(RobotConstants.Lift.idle);
                break;
            case HIGH_BUCKET:
                setTarget(RobotConstants.Lift.highBucket);
                break;
            case LOW_BUCKET:
                setTarget(RobotConstants.Lift.lowBucket);
                break;
            case HIGH_BAR:
                setTarget(RobotConstants.Lift.highBar);
                break;
            case LOW_BAR:
                setTarget(RobotConstants.Lift.lowBar);
                break;
            case SUB_INTAKING:
                setTarget(RobotConstants.Lift.submersible);
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

    public void incrementSlides(double input) {
        setTarget((int)(rightSlide.getCurrentPosition()+input));
    }

    public void altZeroPowerBehavior(){
        if (leftSlide.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE){
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    public int getPosition(){
        return rightSlide.getCurrentPosition();
    }
}
