package org.firstinspires.ftc.teamcode.Tests;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/8/17.
 */

@TeleOp(name = "Button Test", group = "Test")

public class ButtonTest extends LinearOpMode {

    CRServo button;
    ButtonPusherTask buttonPusherTask;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        buttonPusherTask.start();
        buttonPusherTask.outPusher();
        sleep(2000);
        buttonPusherTask.pushButton();
        sleep(2000);
        buttonPusherTask.inPusher();
        sleep(2000);
        buttonPusherTask.running = false;
    }
    public void initialize() {
        button = hardwareMap.crservo.get("button");
        button.setPower(ButtonPusherTask.zeroPower);
        // initialize these tasks to stop them from moving
        CapBallTask capBallTask = new CapBallTask(this);
        IntakeTask intakeTask = new IntakeTask(this);
        buttonPusherTask = new ButtonPusherTask(this, button);
    }
}