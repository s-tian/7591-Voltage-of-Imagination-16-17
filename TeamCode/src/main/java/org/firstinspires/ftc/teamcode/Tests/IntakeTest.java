package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/13/17.
 * Intake Test
 */


@TeleOp(name = "Intake Test", group = "Test")

public class IntakeTest extends LinearOpMode {

    IntakeTask intakeTask;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean oscillate = false;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeTask = new IntakeTask(this);
        waitForStart();
        intakeTask.start();
        powerSweeper(-1, 2000);
        sleep (2000);
        powerSweeper(1, 1000);
        sleep(1000);
    }

    public void powerSweeper(double power, int time) {
        intakeTask.power = power;
        intakeTask.sweepTime = time;
    }

}