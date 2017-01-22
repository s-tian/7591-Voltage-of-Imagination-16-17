package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Howard on 1/22/17.
 */

public abstract class TaskThread extends Thread{
    public volatile boolean running = true;
    public LinearOpMode opMode;
    public volatile boolean teleOp = false;
    public static volatile double voltage = 13;
    public static final double EXPECTED_VOLTAGE = 13;

    public final void sleep (int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void calculateVoltage(LinearOpMode opMode) {
        double mc7 = opMode.hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = opMode.hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = opMode.hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = opMode.hardwareMap.voltageSensor.get("flywheels").getVoltage();
        voltage = (mc7 + mc6 + mc3 + mc2) / 4;
    }

    public abstract void initialize();

}
