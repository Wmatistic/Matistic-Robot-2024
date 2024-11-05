package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;


@TeleOp
public class Slides extends OpMode {

    private DcMotorEx leftSlide, rightSlide;

    private GamepadEx driver;

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.Lift.leftSlide);
        rightSlide = hardwareMap.get(DcMotorEx.class, RobotConstants.Lift.rightSlide);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        driver.readButtons();


        if (driver.getButton(GamepadKeys.Button.DPAD_UP)){
            setSlidePower(0.75);
        } else if (driver.getButton(GamepadKeys.Button.DPAD_DOWN)){
            setSlidePower(-0.75);
        } else {
            setSlidePower(0);
        }
    }

    public void setSlidePower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }
}
