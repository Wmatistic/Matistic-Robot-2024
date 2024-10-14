package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.State;

public class Robot {

    public Mecanum drivetrain;
    public Lift lift;
    public Arm arm;

    private State state;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        state = State.IDLE;
    }

    public void setPosition(State state){
        lift.setPosition(state);

        this.state = state;
    }

    public State getState(){
        return state;
    }

}
