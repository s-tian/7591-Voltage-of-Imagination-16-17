package org.firstinspires.ftc.teamcode.opmodes;

/**
 * Created by Howard on 12/7/16.
 */

import com.qualcomm.hardware.adafruit.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;


/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoRed - Backwards!", group = "Tests")

public class AutoRed extends LinearOpMode {
    boolean missedLineUp = false;
    int parallelAngle;
    int delay = 200;
    boolean missed = false, pickUp = false, detectBlue1 = false;
    int shootRotation = 90;
    int shootRotation2 = 136;
    int sralt = 141;
    final int capBallRotation = -180;
    final int pickUpRotation = 152;
    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;
    int betweenBeacon = 32;
    int bbalt = 36;
    int bbalt2 = 25;
    int angle = 35;
    double shootPower = 0.7;
    double spalt = 0.62;
    ColorSensor colorSensorTop, colorSensorBottom;
    VOIColorSensor voiColorSensorTop, voiColorSensorBottom;
    Servo forkLeft, forkRight, button;
    VOIImu imu;
    VOISweeper sweeper;
    CRServo sweeper1, sweeper2;
    BNO055IMU adaImu;
    public FlywheelTask flywheelTask;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft; //sweeper, conveyor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void runOpMode() {
        System.out.println("Hello world");
        initialize();
        //options();
        waitForStart();
        if (pickUp){
            pickUpBall();
            lineUpToWall(40);
        } else {
            lineUpToWall(50);
        }
        drivePushButton();
        drivePushButton2();
        if (missed){
            flywheelTask.setFlywheelPow(spalt);
            checkFirst();
            moveFromWall();
            coolDown();
            hitCapBall();
        }else {
            moveFromWall2();
            coolDown();
            hitCapBall2();
        }
    }

    public void pause(){
        driveTrain.stopAll();
        sleep(delay);
    }

    public void initialize(){
        timer.reset();
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        sweeper = new VOISweeper(sweeper1, sweeper2);
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
        button = hardwareMap.servo.get("button");
        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        forkLeft.setPosition(0.8);
        forkRight.setPosition(0.12);
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        button.setPosition(0.1);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.start();

        parallelAngle = VOIImu.addAngles(imu.getAngle(), angle);
    }

    public void lineUpToWall(int distance) {
        driveTrain.moveBackwardNInch(0.2, 0.5, 10, false, false);
        driveTrain.moveBackwardNInch(0.6, distance-0.5, 10, false, true);
        //pause();
        driveTrain.powerAllMotors(-0.15);
        boolean detectColor = false;
        timer.reset();
        int initialTicks = frontRight.getCurrentPosition();
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                if (voiColorSensorBottom.isWhite()) {
                    detectColor = true;
                }

                timer.reset();
                if (frontRight.getCurrentPosition() - initialTicks < -35 * driveTrain.TICKS_PER_INCH_FORWARD ){
                    driveTrain.stopAll();
                    break;
                }
            }
        }
        //pause();
        // align with wall
        if (detectColor) {
            driveTrain.rotateDegreesPrecision(angle);
        } else {
            driveTrain.rotateDegreesPrecision(-90);
            missedLineUp = true;
            driveTrain.moveBackwardNInch(0.2, 3, 3, false, true);
            int rotationAngle = VOIImu.subtractAngles(parallelAngle, imu.getAngle(), true);
            driveTrain.rotateDegreesPrecision(rotationAngle);
        }

        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 40, 10, true, true);
        //pause();

    }

    public void drivePushButton() {
        // move backwards to get behind beacon
        if (!missedLineUp) {
            driveTrain.moveForwardNInch(0.2, 6, 10, false, true);
        }
        pause();
        // move backward
        driveTrain.powerAllMotors(-0.1);
        boolean detectColor = false;
        int initialTicks = backRight.getCurrentPosition();
        timer.reset();
        timer2.reset();
        missed = false;
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                detectColor = voiColorSensorTop.isRed();
                timer.reset();
                if (voiColorSensorTop.isBlue())  {
                    detectBlue1 = true;
                    initialTicks = backRight.getCurrentPosition();
                }

                if ((detectBlue1 && backRight.getCurrentPosition()-initialTicks < -7*driveTrain.TICKS_PER_INCH_FORWARD) || timer2.time()>3000) {
                    betweenBeacon = bbalt2;
                    missed = true;
                    return;
                }
            }
        }

        if (!detectBlue1)
            betweenBeacon = bbalt;
        driveTrain.stopAll();
        // move forward to align button pusher with beacon button and push
        correctionStrafe();
        pause();
        pushButton();
    }

    public void drivePushButton2() {
        driveTrain.moveBackwardNInch(0.4,betweenBeacon, 10, false, true);
        flywheelTask.setFlywheelPow(shootPower);
        //pause();
        correctionStrafe(0.5);
        boolean detectColor = false;
        driveTrain.powerAllMotors(-0.1);
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        while (!detectColor && opModeIsActive() && timeout.time() < 5000) {
            if (timer.time() > 30) {
                // determine which side of beacon
                if (voiColorSensorTop.isBlue()){
                    shootRotation2 = sralt;
                }
                detectColor = voiColorSensorTop.isRed();
                timer.reset();
            }
        }
        if (timeout.time() >= 5000) {
            giveUpSecond();
        }

        correctionStrafe(0.5);
        pushButton();
    }

    public void moveFromWall(){
        driveTrain.moveLeftNInch(0.6, 6, 10, false, true);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation);
        sleep(200);
        //conveyor.setPower(0.3);
        sweeper.setPower(1);
        sleep(1500);
    }

    public void moveFromWall2 (){
        flywheelTask.setFlywheelPow(shootPower);
        driveTrain.moveLeftNInch(0.6, 8, 10, false, true);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation2);
        //pause();
        sleep(500);
        driveTrain.moveBackwardNInch(0.2, 1, 3,false, false);
        driveTrain.moveBackwardNInch(0.3,12,3,false, false);
        driveTrain.moveBackwardNInch(0.15, 5, 3, false, true);
        sleep(250);
        sweeper.setPower(1);
        sleep(1500);
    }

    public void checkFirst() {
        driveTrain.moveBackwardNInch(0.4,42, 10, false, true);
        correctionStrafe();
        driveTrain.powerAllMotors(-0.15);
        boolean isBlue = false;
        boolean rammedBlue = false;
        while (opModeIsActive() && !voiColorSensorTop.isRed() && !rammedBlue) {
            if (voiColorSensorTop.isBlue() && !isBlue) {
                isBlue = true;
                timer.reset();
            }
            if (isBlue && timer.time() > 1000) {
                rammedBlue = true;
                driveTrain.stopAll();
            }
        }
        if (rammedBlue) {
            correctionStrafe();
            driveTrain.powerAllMotors(-0.1);
            while(opModeIsActive() && !voiColorSensorTop.isBlue()) {
            }
            driveTrain.stopAll();
            correctionStrafe();
            pushButton();
        }
        else {
            driveTrain.stopAll();
            correctionStrafe();
            pushButton();
        }
    }

    public void coolDown() {
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000);
        flywheelTask.setFlywheelPow(0.4);
        sweeper.setPower(0);
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000);
        flywheelTask.setFlywheelPow(0);
        flywheelTask.running = false;
    }

    public void goBackAndPress() {
        driveTrain.powerAllMotors(0.15);
        while (opModeIsActive() && !voiColorSensorTop.isBlue());
        correctionStrafe();
        pushButton();
    }

    public void giveUpSecond() {
        driveTrain.moveForwardNInch(0.3, 5, 5, false, true);
        correctionStrafe(2);
        driveTrain.powerAllMotors(0.2);
        timer.reset();
        while (opModeIsActive() && !voiColorSensorBottom.isWhite() && timer.time() < 10000) {
        }
        if (timer.time() >= 10000) {
            stop();
        }
        correctionStrafe();
        moveFromWall();
        stop();

    }
    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        driveTrain.moveRightNInch(0.2, 5, seconds, false, true);
    }

    public void hitCapBall(){
        int initialDirection = imu.getAngle();
        driveTrain.moveBackwardNInch(1, 50, 10, true, true);
        driveTrain.rotateDegrees((int) (capBallRotation * 0.3), false);
        driveTrain.rotateDegrees((int)((initialDirection-imu.getAngle())*0.25), false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true);
    }

    public void hitCapBall2(){
        driveTrain.moveBackwardNInch(0.25, 36, 10, false, true);
    }

    public void pickUpBall(){
        //sweeper.setPower(1);
        sleep(1500);
        //sweeper.setPower(0);
        driveTrain.moveRightNInch(0.5, 10, 5, false, true);
        driveTrain.rotateDegreesPrecision(pickUpRotation);
    }

    public void pushButton(){
        button.setPosition(1);
        sleep(500);
        button.setPosition(0);
        sleep(500);
    }

    public void options(){
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
            telemetry.addData("Shoot power", shootPower);
            telemetry.addData("Pick up ball?", pickUp);
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

}

