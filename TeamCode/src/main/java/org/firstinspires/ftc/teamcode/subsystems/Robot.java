package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {

    public Mecanum drivetrain;
    public Lift lift;

    private State state;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Mecanum(hardwareMap);
    }

    public void setPosition(State state){
        lift.setPosition(state);

        this.state = state;
    }

    public State getState(){
        return state;
    }

}
