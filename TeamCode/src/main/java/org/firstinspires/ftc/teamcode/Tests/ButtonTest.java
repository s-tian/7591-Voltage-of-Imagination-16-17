package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/8/17.
 */

@TeleOp(name = "Button Test", group = "Test")

public class ButtonTest extends LinearOpMode {

    CRServo button;
    public static double zeroPower = -0.44;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        initPusher();
        sleep(500);
        pushButton();
    }
    public void initialize() {
        button = hardwareMap.crservo.get("button");
        button.setPower(zeroPower);
        CapBallTask capBallTask = new CapBallTask(this);
        IntakeTask intakeTask = new IntakeTask(this);
    }
    public void pushButton() {
        button.setPower(-1);
        sleep(600);
        button.setPower(0.12);
        sleep(600);
        button.setPower(zeroPower);
    }
    public void initPusher() {
        button.setPower(-1);
        sleep(1200);
        button.setPower(zeroPower);
    }

}