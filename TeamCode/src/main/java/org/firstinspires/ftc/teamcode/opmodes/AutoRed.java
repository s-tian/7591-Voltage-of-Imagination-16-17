package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;


/**
 * Created by Howard on 10/28/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoRed", group = "Tests")
/*
If time:
1. Backup for missing white line
2.
3.
 */

public class AutoRed extends LinearOpMode {
    final int delay = 200;
    int shootRotation = 82;
    int sralt = 76;
    final int pickUpRotation = -115;
    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;
    double shootPower = 0.8;
    int betweenBeacon = 35;
    int bbalt = 39;
    boolean pickUp = false;

    BNO055IMU adaImu;
    VOIImu imu;
    ColorSensor colorSensorTop, colorSensorBottom;
    VOIColorSensor voiColorSensorTop, voiColorSensorBottom;
    Servo gate, button;
    CRServo sweeper1, sweeper2;
    VOISweeper sweeper;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, conveyor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    FlywheelTask flywheelTask;

    public void runOpMode() {
        System.out.println("Hello world");
        initialize();
        options();
        waitForStart();
        if (pickUp){
            pickUpBall();
            lineUpToWall(75);
        } else {
            lineUpToWall(90);
        }
        drivePushButton();
        drivePushButton2();
        moveFromWall();
        coolDown();
        hitCapBall();
    }

    public void pause() {
        driveTrain.stopAll();
        sleep(delay);
    }

    public void initialize() {
        timer.reset();
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.start();
        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorSensorBottom = hardwareMap.colorSensor.get("colorBottom");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(topSensorID));//maybe create8bit
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(bottomSensorID));
        voiColorSensorTop = new VOIColorSensor(colorSensorTop, this);
        voiColorSensorBottom = new VOIColorSensor(colorSensorBottom, this);
        gate = hardwareMap.servo.get("gate");
        button = hardwareMap.servo.get("button");
        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        sweeper = new VOISweeper(sweeper1, sweeper2);
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //gyro.calibrate();
        //gyro.resetZAxisIntegrator(); //address is 0x20
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        button.setPosition(0);
        gate.setPosition(0.4);
        driveTrain = new MecanumDriveTrain(backLeft, backRight, frontLeft, frontRight, imu, this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lineUpToWall(int distance) {
        driveTrain.moveForwardNInch(0.2, 1, 2, false);
        driveTrain.moveForwardNInch(0.5, distance, 10, false);
        //pause();
        driveTrain.powerAllMotors(0.15);
        boolean detectColor = false;
        timer.reset();
        while (!detectColor && opModeIsActive()) {
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            if (timer.time() > 30) {
                detectColor = voiColorSensorBottom.isWhite();
                telemetry.addData("Color: ", red + " " + green + " " + blue);
                updateTelemetry(telemetry);
                timer.reset();
            }
        }
        //pause();
        // align with wall
        //driveTrain.rotateDegrees((int) (ramRotation * 0.49), false);
        driveTrain.rotateDegreesPrecision(-150);
        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 40, 10, true);
        //pause();
    }

    public void drivePushButton() {
        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.3, 6, 10, false);

        // move forward until beacon detected

        driveTrain.powerAllMotors(0.12);
        boolean detectColor = false;
        boolean oppositeColor = false;
        timer.reset();
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                int red = colorSensorBottom.red();
                int green = colorSensorBottom.green();
                int blue = colorSensorBottom.blue();
                telemetry.addData("ColorWall: ", red + " " + green + " " + blue);
                telemetry.addData("Red: ", voiColorSensorBottom.isRed());
                telemetry.addData("Blue: ", voiColorSensorBottom.isBlue());
                updateTelemetry(telemetry);

                detectColor = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                timer.reset();
                if (voiColorSensorTop.isBlue())
                    oppositeColor = true;
            }
        }
        // move forward to align button pusher with beacon button and push
        correctionStrafe(0.5);
        if (!oppositeColor) {
            betweenBeacon = bbalt;
        }
        pushButton();
    }

    public void drivePushButton2() {
        driveTrain.moveForwardNInch(0.3, 32, 10, false);
        correctionStrafe(0.5);
        boolean detectColor = false;
        driveTrain.powerAllMotors(0.15);
        timer.reset();
        flywheelTask.setFlywheelPow(shootPower);
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                int red = colorSensorBottom.red();
                int green = colorSensorBottom.green();
                int blue = colorSensorBottom.blue();
                telemetry.addData("ColorWall: ", red + " " + green + " " + blue);
                telemetry.addData("Red: ", voiColorSensorBottom.isRed());
                telemetry.addData("Blue: ", voiColorSensorBottom.isBlue());
                updateTelemetry(telemetry);
                detectColor = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                timer.reset();
                if (voiColorSensorBottom.isBlue()){
                    shootRotation = sralt;
                }
            }
        }
        correctionStrafe(0.5);
        pushButton();
    }

    public void pushButton() {
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
    }

    public void moveFromWall() {
        driveTrain.moveLeftNInch(0.15, 1, 2, false);
        driveTrain.moveLeftNInch(0.3, 4, 7, false);
        driveTrain.moveLeftNInch(0.15, 1, 2, false);
        driveTrain.rotateDegreesPrecision(shootRotation);
        sweeper.setPower(1);
        if (pickUp) {
            sleep(2000);
        } else {
            sleep(1500);
        }
    }

    public void coolDown() {
        timer.reset();
        sleep(1000);
        while (opModeIsActive() && timer.time() < 1000);
        flywheelTask.setFlywheelPow(0.4);
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000);
        flywheelTask.setFlywheelPow(0);
        sweeper.setPower(0);
        flywheelTask.running = false;
    }

    public void correctionStrafe(double seconds) {
        driveTrain.moveRightNInch(0.2, 5, seconds, false);
    }

    public void hitCapBall() {
        driveTrain.moveBackwardNInch(1, 50, 10.2, true);
        driveTrain.rotateDegreesPrecision(300);
        driveTrain.stopAll();
        sleep(100);
        driveTrain.moveBackwardNInch(1, 20, 11.3, true);
    }

    public void options() {
        telemetry.addData("Pick up ball?", pickUp);
        telemetry.update();
        boolean confirmed = false;
        boolean dPadUpPressed = false;
        boolean dPadDownPressed = false;
        while(!confirmed){
            if (gamepad1.a){
                pickUp = true;
            }
            if (gamepad1.b){
                pickUp = false;

            }
            if (gamepad1.dpad_down&& !dPadDownPressed && shootPower > 0){
                dPadDownPressed = true;
                shootPower -= 0.01;
            }
            if (gamepad1.dpad_up && !dPadUpPressed && shootPower < 1){
                dPadUpPressed = true;
                shootPower += 0.01;
            }
            if (!gamepad1.dpad_down){
                dPadDownPressed = false;
            }
            if (!gamepad1.dpad_up){
                dPadUpPressed = false;
            }
            telemetry.addData("Pick up ball?: ", pickUp );
            telemetry.addData("Shoot power", shootPower);
            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Shoot power", shootPower);
                telemetry.addData("Pick up ball?", pickUp);
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                confirmed = true;
            }
        }
    }

    public void pickUpBall(){
        sleep(1500);
        sweeper.setPower(1.0);
        driveTrain.moveLeftNInch(0.5, 10, 5, false);
        driveTrain.rotateDegreesPrecision(pickUpRotation);
    }

}

