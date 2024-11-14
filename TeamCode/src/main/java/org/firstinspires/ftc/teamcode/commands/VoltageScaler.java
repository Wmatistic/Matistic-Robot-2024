package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VoltageScaler {
    private HardwareMap hardwareMap;

    private PIDFController voltagePIDF;

    private double voltage;

    public VoltageScaler(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        voltagePIDF = new PIDFController(RobotConstants.VoltagePID.P, RobotConstants.VoltagePID.I, RobotConstants.VoltagePID.D, RobotConstants.VoltagePID.F);
    }

    public double getVoltageCorrection(){
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        return voltagePIDF.calculate(voltage, RobotConstants.VoltagePID.TARGET_VOLTAGE);
    }

    public double getVoltage(){
        return voltage;
    }
}
