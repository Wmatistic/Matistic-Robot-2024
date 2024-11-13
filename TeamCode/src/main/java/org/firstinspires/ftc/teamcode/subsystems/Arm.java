package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm implements Subsystem {
    private VoltageScaler voltageScaler;

    private ElapsedTime elapsedTime;

    private CRServo leftArm, rightArm;
    private Servo rotation, claw, wrist;

    private AnalogInput leftArmInput, rightArmInput;

    private PIDFController armPID;
    private double target, wristTarget, rotationTarget, delay;
    private boolean clawOpen;
    private double globalCorrection;

    public Arm (HardwareMap hardwareMap){
        voltageScaler = new VoltageScaler(hardwareMap);

        elapsedTime = new ElapsedTime();

        rightArmInput = hardwareMap.get(AnalogInput.class, "rightArmInput");
        leftArmInput = hardwareMap.get(AnalogInput.class, "leftArmInput");

        rightArm = hardwareMap.crservo.get(RobotConstants.Arm.rightArm);
        leftArm = hardwareMap.crservo.get(RobotConstants.Arm.leftArm);

        armPID = new PIDFController(RobotConstants.Arm.P, RobotConstants.Arm.I, RobotConstants.Arm.D, RobotConstants.Arm.F);

        wrist = hardwareMap.servo.get(RobotConstants.Arm.wrist);
        rotation = hardwareMap.servo.get(RobotConstants.Arm.rotation);
        claw = hardwareMap.servo.get(RobotConstants.Arm.claw);
    }

    public void setPosition(State state){
        switch(state){
            case IDLE:
                setAssembly(RobotConstants.Arm.armIdle, RobotConstants.Arm.wristIdle, RobotConstants.Arm.rotationIdle, false, 0);
                break;
            case SUB_INTAKING:
                setAssembly(RobotConstants.Arm.armSubmersible, RobotConstants.Arm.wristSubmersible, RobotConstants.Arm.rotationIdle, true, 0);
                break;
            case SUB_GRABBING:
                setAssembly(RobotConstants.Arm.armSubmersibleGrab, RobotConstants.Arm.wristSubmerisbleGrab, rotationTarget, false, RobotConstants.Arm.submerisbleDelay);
                elapsedTime.reset();
                break;
            case TRANSFER:
                setAssembly(RobotConstants.Arm.armTransfer, RobotConstants.Arm.wristTransfer, 0, false, 0);
                break;
            case LOW_BAR:
                setAssembly(RobotConstants.Arm.armLowBar, RobotConstants.Arm.wristLowBar, RobotConstants.Arm.rotationIdle, false, 0);
                break;
            case LOW_BAR_SLAM:
                setAssembly(RobotConstants.Arm.armLowBarSlam, RobotConstants.Arm.wristLowBarSlam, RobotConstants.Arm.rotationIdle, true, RobotConstants.Arm.lowBarDelay);
                elapsedTime.reset();
                break;
            case LOW_BUCKET:
            case HIGH_BUCKET:
                setAssembly(RobotConstants.Arm.armScoring, RobotConstants.Arm.wristScoring, RobotConstants.Arm.rotationIdle, false, 0);
        }
    }

    public void updateAssembly(){
        setArmsTarget(target);
        updateArms();

        if (elapsedTime.time() > delay){
            setWrist(wristTarget);
            setRotation(rotationTarget);
            setClaw(clawOpen);
        }

    }

    public void setAssembly(double target, double wristTarget, double rotationTarget, boolean clawOpen, double delay){
        this.target = target;
        this.wristTarget = wristTarget;
        this.rotationTarget = rotationTarget;
        this.clawOpen = clawOpen;
        this.delay = delay;
    }

    public void setClaw(boolean clawOpen){
        if (clawOpen) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public void actuateClaw(){
        if (claw.getPosition() == RobotConstants.Arm.clawClose){
            openClaw();
        } else {
            closeClaw();
        }
    }

    public void closeClaw(){
        clawOpen = false;
        claw.setPosition(RobotConstants.Arm.clawClose);
    }

    public void openClaw(){
        clawOpen = true;
        claw.setPosition(RobotConstants.Arm.clawOpen);
    }

    public void incrementRotation(double increment){
        rotationTarget += increment;
    }

    public void setRotation(double rotationTarget) {
        rotation.setPosition(rotationTarget);
    }

    public void setWrist(double target){
        wrist.setPosition(target);
    }

    public double getArmPosition(){
        return rightArmInput.getVoltage() / 3.3;
    }

    public void setArmsTarget(double target) {
        this.target = target;
    }

    public void updateArms() {
        // Commented for now but maybe needed
        double voltageCorrection = voltageScaler.getVoltageCorrection();

        double correction;
        correction = -armPID.calculate(getArmPosition(), target + voltageCorrection);

        globalCorrection = correction;

        rightArm.setPower(correction);
        leftArm.setPower(correction);
    }

    public double getCorrection() {
        return globalCorrection;
    }
}
