package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class Tuning extends OpMode {

    private Lift lift;

    private GamepadEx driver, operator;

    private int slidePosition;

    @Override
    public void init() {
        telemetry.addLine("Initializing");
        telemetry.update();

        lift = new Lift(hardwareMap);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        telemetry.addLine("Running");
        telemetry.update();

        slidePosition = lift.getPosition();

        telemetry.addData("Slide Position: ", slidePosition);
        telemetry.update();

        if(driver.wasJustPressed(GamepadKeys.Button.A)){
          lift.altZeroPowerBehavior();
        }
    }
}
