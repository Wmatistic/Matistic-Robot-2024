package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.State;

public class HorizontalExtension implements Subsystem {

    private Servo rail, leftExtension, rightExtension;

    private double target = 0.0;

    public HorizontalExtension(HardwareMap hardwareMap) {
        rail = hardwareMap.servo.get(RobotConstants.HorizontalExtension.rail);
        leftExtension = hardwareMap.servo.get(RobotConstants.HorizontalExtension.leftExtension);
        rightExtension = hardwareMap.servo.get(RobotConstants.HorizontalExtension.rightExtension);
    }

    public void setPosition(State state){
        switch (state){
            case IDLE:
                setExtension(RobotConstants.HorizontalExtension.idle);
                break;
            case SUB_INTAKING:
                setExtension(RobotConstants.HorizontalExtension.subIntaking);
                break;
            case HIGH_BUCKET:
            case LOW_BUCKET:
            case HIGH_BUCKET_SLAM:
                setExtension(RobotConstants.HorizontalExtension.idle);
                break;
            case HIGH_BAR:
            case HIGH_BAR_SLAM:
            case LOW_BAR:
                setExtension(RobotConstants.HorizontalExtension.scoring);
                break;
        }
    }

    public void setExtension(double target){
        this.target = target;

        if (target * RobotConstants.HorizontalExtension.railMax <= RobotConstants.HorizontalExtension.railMax) {
            rail.setPosition(target * RobotConstants.HorizontalExtension.railMax);
            leftExtension.setPosition(0);
            rightExtension.setPosition(0);
        } else {
            rail.setPosition(RobotConstants.HorizontalExtension.railMax);
            leftExtension.setPosition((target - 1) * RobotConstants.HorizontalExtension.slideMax);
            rightExtension.setPosition((target - 1) * RobotConstants.HorizontalExtension.slideMax);

        }
    }

    public void incrementExtension(double input){
        setExtension(target + input);
    }

    public double getExtensionPosition(){
        return target;
    }

}
