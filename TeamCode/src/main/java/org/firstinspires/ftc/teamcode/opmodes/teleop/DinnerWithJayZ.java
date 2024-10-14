package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.State;
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

            // --------------------------- GLOBAL CONTROLS --------------------------- //

        if(driver.wasJustPressed(GamepadKeys.Button.Y)){
            bot.drivetrain.resetHeading();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.X)){
            bot.drivetrain.changeMode();
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
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    bot.setPosition(State.HIGH_BAR);
                }
                if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                    bot.setPosition(State.LOW_BAR);
                }

                break;
            case SUB_INTAKING:

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
            case GROUND_INTAKING:

                // Return To Default
                if (!driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                    bot.setPosition(State.IDLE);
                }

                break;
        }
    }
}
