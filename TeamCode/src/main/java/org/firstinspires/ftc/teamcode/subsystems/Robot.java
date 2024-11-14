package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.State;

public class Robot {

    public Mecanum drivetrain;
    public Lift lift;
    public Arm arm;
    public HorizontalExtension horizontalExtension;

    private State state;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        horizontalExtension = new HorizontalExtension(hardwareMap);

        state = State.IDLE;
        setPosition(state);
    }

    public void setPosition(State state){
        lift.setPosition(state);
        arm.setPosition(state);
        horizontalExtension.setPosition(state);

        this.state = state;
    }

    public State getState(){
        return state;
    }

}
