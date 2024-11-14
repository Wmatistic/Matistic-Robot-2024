package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.commands.Poses.blueBucket;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class BlueBucket extends OpMode {

    PinpointDrive drive;
    TelemetryPacket tele;
    SequentialAction path;
    Robot bot;

    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap, blueBucket.start);
        tele = new TelemetryPacket();

        path = createPath();

        bot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {

    }

    public SequentialAction createPath() {
        return new SequentialAction(
                preloadSpecimen()
        );
    }

    public ParallelAction preloadSpecimen(){
        return new ParallelAction(

        );
    }
}
