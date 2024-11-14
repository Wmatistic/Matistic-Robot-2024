package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.VoltageReader;
import org.firstinspires.ftc.teamcode.commands.VoltageScaler;

public class Mecanum implements Subsystem {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;
    private IMU imu;
    private Mode mode;
    private Drive_Mode driveMode;

    private final VoltageScaler voltageScaler;

    private double leftFrontLastPower = 0;
    private double leftRearLastPower = 0;
    private double rightFrontLastPower = 0;
    private double rightRearLastPower = 0;
    private double power = 0;
    private double k = 0.1;
    private long lastZeroTime = 0;
    private static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 75;

    // leftFront, leftRear, rightFront, rightRear
    private final double[] minPowersToOvercomeFriction = new double[] {
            0.115,
            0.115,
            0.115,
            0.115
    };

    enum Mode{FIELD, ROBOT}

    enum Drive_Mode{ANTI_FRICTION, FRICTION}

    public Mecanum(HardwareMap hardwareMap) {
        voltageScaler = new VoltageScaler(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Mecanum.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Mecanum.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Mecanum.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Mecanum.rightFront);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        imu.resetYaw();

        mode = mode.FIELD;
        driveMode = Drive_Mode.FRICTION;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(GamepadEx gamepad) {
        double y = -gamepad.getLeftY();
        double x = -gamepad.getLeftX() * 1.1;
        rx = gamepad.getRightX() * 1.1;

        switch (mode) {
            case FIELD:
                heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

                leftFrontPower = (rotY + rotX + rx);
                leftRearPower = (rotY - rotX + rx);
                rightFrontPower = (rotY - rotX - rx);
                rightRearPower = (rotY + rotX - rx);

                break;
            case ROBOT:

                leftFrontPower = (y + x + rx);
                leftRearPower = (y - x + rx);
                rightFrontPower = (y - x - rx);
                rightRearPower = (y + x - rx);

                break;
        }

        switch(driveMode) {
            case ANTI_FRICTION:
                if (leftFrontPower == 0) {
                    leftFront.setPower(leftFrontPower);
                } else if (leftFrontPower > 0) {
                    leftFront.setPower(leftFrontPower + minPowersToOvercomeFriction[0]);
                } else if (leftFrontPower < 0) {
                    leftFront.setPower(leftFrontPower - minPowersToOvercomeFriction[0]);
                }

                if (leftRearPower == 0) {
                    leftRear.setPower(leftRearPower);
                } else if (leftFrontPower > 0) {
                    leftRear.setPower(leftRearPower + minPowersToOvercomeFriction[1]);
                } else if (leftFrontPower < 0) {
                    leftRear.setPower(leftRearPower - minPowersToOvercomeFriction[1]);
                }

                if (rightFrontPower == 0) {
                    rightFront.setPower(rightFrontPower);
                } else if (rightFrontPower > 0) {
                    rightFront.setPower(rightFrontPower + minPowersToOvercomeFriction[2]);
                } else if (rightFrontPower < 0) {
                    rightFront.setPower(rightFrontPower - minPowersToOvercomeFriction[2]);
                }

                if (rightRearPower == 0) {
                    rightRear.setPower(rightRearPower);
                } else if (rightRearPower > 0) {
                    rightRear.setPower(rightRearPower + minPowersToOvercomeFriction[3]);
                } else if (rightRearPower < 0) {
                    rightRear.setPower(rightRearPower - minPowersToOvercomeFriction[3]);
                }
                break;
            case FRICTION:
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
                break;
        }
    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading(){
        imu.resetYaw();
    }

    public void changeMode(){
        if(mode == Mode.ROBOT){
            mode = Mode.FIELD;
        } else {
            mode = Mode.ROBOT;
        }
    }

    private void setMode(Mode m){
        mode = m;
    }

    public void changeDriveMode(){
        if(driveMode == Drive_Mode.ANTI_FRICTION){
            driveMode = Drive_Mode.FRICTION;
        } else {
            driveMode = Drive_Mode.ANTI_FRICTION;
        }
    }

    public double[] getMotorPowers(){
        return new double[] {
                leftFrontPower,
                leftRearPower,
                rightFrontPower,
                rightRearPower
        };
    }
}
