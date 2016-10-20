package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class DriveTrainTask extends Thread {

    private MecanumDriveTrain driveTrain;
    private LinearOpMode opMode;

    public volatile boolean running = true;

    public DriveTrainTask(LinearOpMode opMode, MecanumDriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {


        }


    }
}
