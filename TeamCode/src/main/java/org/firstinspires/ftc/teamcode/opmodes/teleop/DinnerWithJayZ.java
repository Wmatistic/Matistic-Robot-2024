package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.commands.VoltageReader;

@TeleOp
public class DinnerWithJayZ extends OpMode {

    private Robot bot;

    private GamepadEx driver, operator;
    private VoltageReader voltageReader;
    double slideOverride;

    @Override
    public void init() {
        telemetry.addLine("Initializing");
        telemetry.update();

        bot = new Robot(hardwareMap);
        voltageReader = new VoltageReader(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {

        telemetry.addLine("Running");
        telemetry.addData("Current State: ", bot.getState());

        telemetry.addData("\n Lift Position: ", bot.lift.getPosition());
        telemetry.addData("\n Lift Power: ", bot.lift.getLiftPower());
        telemetry.addData("\n Extension Position: ", bot.horizontalExtension.getExtensionPosition());
        telemetry.addData("\n Arm Position: ", bot.arm.getArmPosition());
        telemetry.addData("\n Arm Correction: ", bot.arm.getCorrection());
        telemetry.addData("\n Slide PID Position Error: ", bot.lift.getPositionError());
        telemetry.addData("\n FL Motor Power: ", bot.drivetrain.getMotorPowers()[0]);
        telemetry.addData("\n BL Motor Power: ", bot.drivetrain.getMotorPowers()[1]);
        telemetry.addData("\n FR Motor Power: ", bot.drivetrain.getMotorPowers()[2]);
        telemetry.addData("\n BR Motor Power: ", bot.drivetrain.getMotorPowers()[3]);
        telemetry.update();

        driver.readButtons();
        operator.readButtons();


        // --------------------------- DRIVER CODE --------------------------- //

        bot.drivetrain.drive(driver);

            // --------------------------- GLOBAL CONTROLS --------------------------- //

        bot.lift.powerSlides();
        bot.arm.updateAssembly();

        if(driver.wasJustPressed(GamepadKeys.Button.Y)){
            bot.drivetrain.resetHeading();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            bot.drivetrain.changeDriveMode();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.X)){
            bot.arm.actuateClaw();
        }


            // --------------------------- SUBMERSIBLE LAYER --------------------------- //

        switch (bot.getState()){
            case IDLE:

                // Go To Different States
                if(driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                    bot.setPosition(State.GROUND_INTAKING);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                    bot.setPosition(State.SUB_INTAKING);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    bot.setPosition(State.HIGH_BUCKET);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    bot.setPosition(State.LOW_BUCKET);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                    bot.setPosition(State.HIGH_BAR);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    bot.setPosition(State.LOW_BAR);
                }

                if(driver.wasJustPressed(GamepadKeys.Button.A)){
                    bot.lift.resetEncoder();
                }

                break;
            case SUB_INTAKING:
            case SUB_GRABBING:

                // Return To Default
                if(driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                    bot.setPosition(State.IDLE);
                }

                if(driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                    if(bot.getState() == State.SUB_INTAKING) {
                        bot.setPosition(State.SUB_GRABBING);
                    } else if (bot.getState() == State.SUB_GRABBING){
                        bot.setPosition(State.SUB_INTAKING);
                    }
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    bot.arm.incrementRotation(0.1);
                } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                    bot.arm.incrementRotation(-0.1);
                }

                // Slowly Extend or Retract Extension
//                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
//                    bot.horizontalExtension.incrementExtension(0.01);
//                }
//                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
//                    bot.horizontalExtension.incrementExtension(-0.01);
//                }

                // Slowly Raise or Lower Slides
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    bot.lift.incrementSlides(1);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    bot.lift.incrementSlides(-1);
                }

                break;
            case HIGH_BUCKET:

                // Return To Default
                if(driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                    bot.setPosition(State.IDLE);
                }

                // Slowly Raise or Lower Slides
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    bot.lift.incrementSlides(1);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    bot.lift.incrementSlides(-1);
                }

                break;

            case LOW_BUCKET:

                // Return To Default
                if(driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                    bot.setPosition(State.IDLE);
                }

                if(driver.wasJustPressed(GamepadKeys.Button.X)){
                    bot.arm.openClaw();
                }

                break;

            case LOW_BAR:
            case LOW_BAR_SLAM:

                // Return To Default
                if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                    bot.setPosition(State.IDLE);
                }

                if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                    if(bot.getState() == State.LOW_BAR) {
                        bot.setPosition(State.LOW_BAR_SLAM);
                    } else if (bot.getState() == State.LOW_BAR_SLAM){
                        bot.setPosition(State.LOW_BAR);
                    }
                }

                break;
        }
    }
}
