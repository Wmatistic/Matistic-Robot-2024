package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.commands.Poses.blueBucket;
import org.firstinspires.ftc.teamcode.commands.RobotActions;
import org.firstinspires.ftc.teamcode.commands.State;

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
        botActions.ArmAssembly().run(tele);
        botActions.SlidePID().run(tele);

        path.run(tele);
    }

    public SequentialAction createPath() {
        return new SequentialAction(
                preloadSpecimen(), firstBucketCycle(), secondBucketCycle(), thirdBucketCycle()
        );
    }

    public ParallelAction preloadSpecimen(){
        return new ParallelAction(
            botActions.setRobotState(State.HIGH_BAR),

            new SequentialAction(
                    drive.actionBuilder(blueBucket.start)
                            .setReversed(true)
                            .lineToY(blueBucket.highBar.position.y)
                            .waitSeconds(0.5)
                            .build(),

                    botActions.setRobotState(State.HIGH_BAR_SLAM),

                    new SleepAction(0.5),

                    botActions.setRobotState(State.IDLE),

                    new SleepAction(0.5)
            )
        );
    }

    public SequentialAction firstBucketCycle() {
        return new SequentialAction(
            botActions.setRobotState(State.IDLE),

            drive.actionBuilder(blueBucket.highBar)

                .setReversed(false)
                .splineToLinearHeading(blueBucket.firstSample, Math.toRadians(90))

                .build(),

            botActions.setRobotState(State.SUB_INTAKING),

            new SleepAction(0.8),

            botActions.setRobotState(State.SUB_GRABBING),

            new SleepAction(0.3),

            botActions.setRobotState(State.IDLE),

            new SleepAction(0.3),

            drive.actionBuilder(blueBucket.firstSample)

                .afterTime(0.1, botActions.setRobotState(State.HIGH_BUCKET))

                .setReversed(false)
                .splineToLinearHeading(blueBucket.bucket, Math.toRadians(178))

                .build(),

            new SleepAction(1),

            botActions.setRobotState(State.HIGH_BUCKET_SLAM),

            new SleepAction(0.2),

            botActions.setClaw(true),

            new SleepAction(0.3),

            botActions.setRobotState(State.IDLE),

            new SleepAction(1)
        );
    }

    public SequentialAction secondBucketCycle() {
        return new SequentialAction(

                drive.actionBuilder(blueBucket.bucket)

                        .setReversed(true)
                        .splineToLinearHeading(blueBucket.secondSample, Math.toRadians(180))

                        .build(),

                botActions.setRobotState(State.SUB_INTAKING),

                new SleepAction(1.0),

                botActions.setRobotState(State.SUB_GRABBING),

                new SleepAction(0.3),

                botActions.setRobotState(State.IDLE),

                new SleepAction(0.3),

                drive.actionBuilder(blueBucket.secondSample)

                        .afterTime(0.1, botActions.setRobotState(State.HIGH_BUCKET))

                        .setReversed(false)
                        .splineToLinearHeading(blueBucket.bucket, Math.toRadians(178))

                        .build(),

                new SleepAction(1),

                botActions.setRobotState(State.HIGH_BUCKET_SLAM),

                new SleepAction(0.2),

                botActions.setClaw(true),

                new SleepAction(0.3),

                botActions.setRobotState(State.IDLE),

                new SleepAction(1)
        );
    }

    public SequentialAction thirdBucketCycle() {
        return new SequentialAction(

                drive.actionBuilder(blueBucket.bucket)

                        .setReversed(true)
                        .splineToLinearHeading(blueBucket.thirdSample, Math.toRadians(180))

                        .build(),

                botActions.setRobotState(State.SUB_INTAKING),
                botActions.setClawRotation(0.6),

                new SleepAction(1.0),

                botActions.setRobotState(State.SUB_GRABBING),

                new SleepAction(0.3),

                botActions.setRobotState(State.IDLE),

                new SleepAction(0.3),

                drive.actionBuilder(blueBucket.thirdSample)

                        .afterTime(0.1, botActions.setRobotState(State.HIGH_BUCKET))

                        .setReversed(false)
                        .splineToLinearHeading(blueBucket.bucket, Math.toRadians(178))

                        .build(),

                new SleepAction(1),

                botActions.setRobotState(State.HIGH_BUCKET_SLAM),

                new SleepAction(0.2),

                botActions.setClaw(true),

                new SleepAction(0.3),

                botActions.setRobotState(State.IDLE),

                new SleepAction(1)
        );
    }
}
