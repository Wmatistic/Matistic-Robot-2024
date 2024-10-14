package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class DinnerWithJayZ extends OpMode {

    private Robot bot;

    private GamepadEx driver, operator;

    @Override
    public void init() {
        telemetry.addLine("Initializing");
        telemetry.update();

        bot = new Robot(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();


        // --------------------------- DRIVER CODE --------------------------- //
        bot.drivetrain.drive(driver);

        if(driver.wasJustPressed(GamepadKeys.Button.Y)){
            bot.drivetrain.resetHeading();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.X)){
            bot.drivetrain.changeMode();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            bot.setPosition(State.SUBINTAKING);
        }

    }
}
