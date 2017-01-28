package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/24/17.
 */

@TeleOp(name = "RotationTest")

public class RotationTest extends LinearOpMode {
    public static double minPow = 0.03;
    public static double initPow = 0.3;
    MecanumDriveTrain driveTrain;
    public static int coolDown = 1000;
    public static int angle = 90;
    VOIImu imu;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() {
        initialize();
        chooseSpeed();
        telemetry.addData("Confirmed", "");
        telemetry.update();
        waitForStart();
        timer.reset();
        boolean confirmed = false;
        while (!confirmed) {

        }
        driveTrain.startRotation(1);
        driveTrain.startRotation(MecanumDriveTrain.minRotPow);
        sleep(1000);
        driveTrain.stopAll();
//        System.out.println("Before: " + imu.getAngle());
//        driveTrain.rotateDegrees(angle, initPow, true);
//        driveTrain.rotateToAngle(0);
//        System.out.println("Time: " + timer.time());
//        System.out.println("Immediately after: " + imu.getAngle());
//        sleep(100);
//        System.out.println("100 ms: " + imu.getAngle());
//        sleep(100);
//        System.out.println("200 ms: " + imu.getAngle());
//        sleep(100);
//        System.out.println("300 ms: " + imu.getAngle());



    }

    public void initialize() {
        new CapBallTask(this);
        new ButtonPusherTask(this);
        new IntakeTask(this);
        driveTrain = new MecanumDriveTrain(this);
        BNO055IMU adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);

    }

    public void chooseSpeed() {
        boolean confirmed = false;
        boolean up = false;
        boolean down = false;
        boolean right = false;
        boolean left = false;
        while (!confirmed) {
            if (gamepad1.dpad_up && !up) {
                up = true;
                angle += 3;
            }
            if (gamepad1.dpad_down && !down) {
                down = true;
                angle -= 3;
            }
            if (gamepad1.dpad_right && !right) {
                right = true;
                initPow += 0.01;
            }
            if (gamepad1.dpad_left && !left) {
                left = true;
                initPow -= 0.01;
            }
            if (!gamepad1.dpad_down) {
                down = false;
            }
            if (!gamepad1.dpad_up) {
                up = false;
            }
            if (!gamepad1.dpad_right) {
                right = false;
            }
            if (!gamepad1.dpad_left) {
                left = false;
            }
            telemetry.addData("angle", angle);
            telemetry.addData("initPow", initPow);

            if ((gamepad1.left_stick_button && gamepad1.right_stick_button) || opModeIsActive()) {
                confirmed = true;
                telemetry.addData("Confirmed", "");
            }
            telemetry.update();
        }
    }

    public void runRotations() {
        boolean cont = true;
        while (cont && opModeIsActive()) {
            double before = imu.getAngle();

            System.out.println("Before: " + before);
            timer.reset();
            driveTrain.rotateDegreesPrecision(angle, initPow);
            System.out.println("Time: " + timer.time());
            System.out.println("Immediately after: " + imu.getAngle());
            sleep(500);
            System.out.println("500 ms after stop: " + imu.getAngle());
            sleep(500);
            double stopAngle = imu.getAngle();
            System.out.println("One sec after stop: " + stopAngle);
            System.out.println("Difference: " + VOIImu.subtractAngles(stopAngle, VOIImu.addAngles(before, angle)));
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    chooseSpeed();
                    telemetry.addData("Confirmed", "");
                    telemetry.update();
                    break;
                }
                if (gamepad1.b) {
                    return;
                }
            }
        }
    }

    public void rotateWithoutStop() {
        driveTrain.startRotation(1);
        driveTrain.startRotation(MecanumDriveTrain.minRotPow);
        sleep(1000);
        driveTrain.stopAll();
    }

    public void rotateWithStop() {

    }
}
