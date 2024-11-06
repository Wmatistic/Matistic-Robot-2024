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

public class Mecanum implements Subsystem {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;
    private IMU imu;
    private Mode mode;

    enum Mode{FIELD, ROBOT}

    public Mecanum(HardwareMap hardwareMap) {

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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(GamepadEx gamepad) {
        double y = -gamepad.getLeftY();
        double x = -gamepad.getLeftX() * 1.1;
        rx = gamepad.getRightX();

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

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
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
}
