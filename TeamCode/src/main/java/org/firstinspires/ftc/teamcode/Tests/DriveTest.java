package org.firstinspires.ftc.teamcode.Tests;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;


/**
 * Created by bunnycide on 11/21/16.
 */

@TeleOp(name = "Drive Test", group = "Test")

public class DriveTest extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    VOIImu imu;
    BNO055IMU adaImu;
    MecanumDriveTrain driveTrain;
    static double power = 0.5, distance = 60;
    public static MecanumDriveTrain.DIRECTION dir = MecanumDriveTrain.DIRECTION.FORWARD;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //chooseDirection();
        waitForStart();
        driveTrain.driveToPosition(2000,2000,2000,2000);

    }

    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(this);

        Servo guide = hardwareMap.servo.get("guide");
        guide.setPosition(ButtonPusherTask.upPosition);
        new ButtonPusherTask(this);
        new IntakeTask(this);
        new CapBallTask(this);

    }

    public void printTicks() {
        System.out.println("br " + backRight.getCurrentPosition());
        System.out.println("bl " + backLeft.getCurrentPosition());
        System.out.println("fr " + frontRight.getCurrentPosition());
        System.out.println("fl " + frontLeft.getCurrentPosition());
    }

    public void chooseDirection() {
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean rBumper = false;
        boolean lBumper = false;
        boolean confirmed = false;
        while (!confirmed) {
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                MecanumDriveTrain.ACF += 0.0001;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                MecanumDriveTrain.ACF -= 0.0001;
            }
            if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                MecanumDriveTrain.changeFactor += 0.001;
            }
            if (!gamepad1.x) {
                xPressed = false;
            }
            if (gamepad1.y && !yPressed) {
                yPressed = true;
                MecanumDriveTrain.changeFactor -= 0.001;
            }
            if (!gamepad1.y) {
                yPressed = false;
            }
            if (gamepad1.left_bumper && !lBumper) {
                lBumper = true;
                distance -= 1;
            }
            if (!gamepad1.left_bumper) {
                lBumper = false;
            }
            if (gamepad1.right_bumper && !rBumper) {
                rBumper = true;
                distance += 1;
            }
            if (!gamepad1.right_bumper) {
                rBumper = false;
            }
            telemetry.addData("ACF", MecanumDriveTrain.ACF);
            telemetry.addData("changeFactor", MecanumDriveTrain.changeFactor);
            telemetry.addData("stallTime", MecanumDriveTrain.stallTime);
            telemetry.addData("Choose Direction", dir);
            telemetry.update();
            if (gamepad1.dpad_up) {
                dir = MecanumDriveTrain.DIRECTION.FORWARD;
            } else if (gamepad1.dpad_down) {
                dir = MecanumDriveTrain.DIRECTION.BACKWARD;
            } else if (gamepad1.dpad_left) {
                dir = MecanumDriveTrain.DIRECTION.LEFT;
            } else if (gamepad1.dpad_right) {
                dir = MecanumDriveTrain.DIRECTION.RIGHT;
            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                confirmed = true;
            }
        }
        System.out.println(confirmed);
        telemetry.addData("Direction", dir);
        telemetry.addData("Confirmed!", "");
        telemetry.update();
    }

    public void testDrive() {
        switch (dir) {
            case FORWARD:
                driveTrain.moveForwardNInch(power, distance, 10, false, true, false);
                sleep(1000);
                driveTrain.moveBackwardNInch(power, distance, 10, false, true, false);
                break;
            case RIGHT:
                driveTrain.moveRightNInch(power, distance, 10, false, true);
                sleep(1000);
                driveTrain.moveLeftNInch(power, distance, 10, false, true);
                break;

            case LEFT:
                driveTrain.moveLeftNInch(power, distance, 10, false, true);
                sleep(1000);
                driveTrain.moveRightNInch(power, distance, 10, false, true);

                break;
            case BACKWARD:
                driveTrain.moveBackwardNInch(power, distance, 10, false, true, false);
                sleep(1000);

                driveTrain.moveForwardNInch(power, distance, 10, false, true, false);

                break;

        }
        printTicks();
        sleep(500);
        printTicks();
    }

    public void chooseSpeed() {

    }

}