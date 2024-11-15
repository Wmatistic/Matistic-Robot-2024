package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.commands.Poses;
import org.firstinspires.ftc.teamcode.commands.Poses.blueBucket;
import org.firstinspires.ftc.teamcode.commands.RobotActions;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class BlueBucket extends OpMode {

    PinpointDrive drive;
    TelemetryPacket tele;
    SequentialAction path;
    RobotActions botActions;

    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap, blueBucket.start);
        tele = new TelemetryPacket();
        botActions = new RobotActions(hardwareMap, drive);

        path = createPath();
    }

    @Override
    public void loop() {
        botActions.ArmPID().run(tele);
        botActions.SlidePID().run(tele);

        path.run(tele);
    }

    public SequentialAction createPath() {
        return new SequentialAction(
                preloadSpecimen()
        );
    }

    public ParallelAction preloadSpecimen(){
        return new ParallelAction(
            botActions.setRobotState(State.HIGH_BAR),
            new SequentialAction(
                    drive.actionBuilder(blueBucket.start)
                            .setReversed(true)
                            .splineTo(blueBucket.blueBucket, Math.toRadians(45))

                            .build()
            )
        );
    }
}
