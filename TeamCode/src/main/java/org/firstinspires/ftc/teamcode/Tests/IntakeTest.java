package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/13/17.
 */


@TeleOp(name = "Intake Test", group = "Test")
@Disabled

public class IntakeTest extends LinearOpMode {

    IntakeTask intakeTask;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeTask = new IntakeTask(this);
        waitForStart();
        intakeTask.start();
        powerSweeper(1, 2000);
        sleep (2000);
    }

    public void powerSweeper(double power, int time) {
        intakeTask.power = power;
        intakeTask.sweepTime = time;
    }
}