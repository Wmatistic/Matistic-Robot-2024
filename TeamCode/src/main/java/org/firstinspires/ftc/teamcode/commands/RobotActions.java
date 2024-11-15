package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;
import androidx.core.view.ActionProvider;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class RobotActions {
    public Robot bot;
    public PinpointDrive drive;

    public RobotActions(HardwareMap hardwareMap, PinpointDrive drive) {
        bot = new Robot(hardwareMap);

        this.drive = drive;
    }

    public Action SlidePID() {
        return new SlidePIDLoop();
    }

    public Action ArmPID() {
        return new ArmPIDLoop();
    }

    public Action setRobotState(State state) {
        return new setRobotState(state);
    }

    public class setRobotState implements Action {
        State state;
        public setRobotState(State state) {
            this.state = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.setPosition(state);
            return false;
        }
    }

    public class SlidePIDLoop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.lift.powerSlides();

            return false;
        }
    }

    public class ArmPIDLoop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.arm.updateArms();

            return false;
        }
    }
}
