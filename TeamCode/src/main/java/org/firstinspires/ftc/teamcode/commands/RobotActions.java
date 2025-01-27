package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

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

    public Action ArmAssembly() {
        return new ArmAssemblyLoop();
    }

    public Action setRobotState(State state) {
        return new setRobotState(state);
    }

    public Action setClaw(boolean clawOpen) { return new setClaw(clawOpen); }

    public Action setClawRotation(double clawRotation) { return new setClawRotation(clawRotation); }

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

    public class setClawRotation implements Action {
        double clawRotation;

        public setClawRotation(double clawRotation) { this.clawRotation = clawRotation; }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.arm.setRotationTarget(clawRotation);
            return false;
        }
    }

    public class setClaw implements Action {
        boolean clawOpen;
        public setClaw(boolean clawOpen) { this.clawOpen = clawOpen; }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.arm.setClawActuate(clawOpen);
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

    public class ArmAssemblyLoop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.arm.updateAssembly();

            return false;
        }
    }
}
