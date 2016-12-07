package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
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
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;

import static java.lang.Thread.sleep;

/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoBlue", group = "Tests")

public class AutoBlue extends LinearOpMode {
    int delay = 200;
    boolean missed = false, pickUp = false, detectRed1 = false;
    int shootRotation = 95;
    int shootRotation2 = 40;
    int sralt = 37;
    final int capBallRotation = -180;
    final int pickUpRotation = 152;
    final boolean REDTEAM = false;
    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;
    int betweenBeacon = 31;
    int bbalt = 37;
    int angle = -35;
    double shootPower = 0.82;
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

    }

    public void lineUpToWall(int distance) {
        driveTrain.moveForwardNInch(0.2, 1, 10, false);
        driveTrain.moveForwardNInch(0.5, distance-1, 10, false);
        //pause();
        driveTrain.powerAllMotors(0.15);
        boolean detectColor = false;
        timer.reset();
        int initialTicks = frontRight.getCurrentPosition();
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                detectColor = voiColorSensorBottom.isWhite();

                timer.reset();
                if (frontRight.getCurrentPosition() - initialTicks > 35 * driveTrain.TICKS_PER_INCH_FORWARD ){
                    driveTrain.moveBackwardNInch(0.3, 10, 8, false);

                    break;
                }
            }
        }
        //pause();
        // align with wall
        if (detectColor)

            driveTrain.rotateDegreesPrecision(angle);
        else
            driveTrain.rotateDegreesPrecision(-90);

        //pause();
        // ram into wall to straighten out
        driveTrain.moveRightNInch(1, 40, 10, true);
        //pause();

    }

    public void drivePushButton() {

        // move backwards to get behind beacon
        driveTrain.moveBackwardNInch(0.4, 4, 10, false);

        // move forward until beacon detected
        pause();
        driveTrain.powerAllMotors(0.1);
        boolean detectColor = false;
        int initialTicks = backRight.getCurrentPosition();
        timer.reset();
        timer2.reset();
        missed = false;
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                detectColor = voiColorSensorTop.isBlue();
                timer.reset();
                if (voiColorSensorTop.isRed()) detectRed1 = true;

                if (backRight.getCurrentPosition()-initialTicks > 20*driveTrain.TICKS_PER_INCH_FORWARD || timer2.time()>5000) {
                    //pause();
                    missed = true;
                    return;
                }
            }
        }

        if (!detectRed1)
            betweenBeacon = bbalt;
        driveTrain.stopAll();
        //pause();
        // move forward to align button pusher with beacon button and push
        //pause();
        correctionStrafe();
        pause();
        pushButton();
    }

    public void drivePushButton2() {
        if (!missed) {
            driveTrain.moveForwardNInch(0.3,betweenBeacon, 10, false);
        } else {
            driveTrain.moveForwardNInch(0.3, betweenBeacon -18, 10, false);
        }
        flywheelTask.setFlywheelPow(shootPower);
        //pause();
        correctionStrafe(4);
        boolean detectColor = false;
        driveTrain.powerAllMotors(0.1);
        timer.reset();

        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                // determine which side of beacon
                if (voiColorSensorTop.isRed()){
                    shootRotation2 = sralt;
                }
                detectColor = voiColorSensorTop.isBlue();
                timer.reset();
            }
        }
        correctionStrafe(2);
        pushButton();
    }

    public void moveFromWall(){
        driveTrain.moveLeftNInch(0.6, 8, 10, false);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation);
        driveTrain.moveForwardNInch(0.6, 2, 10, false);
        sleep(200);
        //pause();
        //conveyor.setPower(0.3);
        sweeper.setPower(1);
        sleep(2500);

    }

    public void moveFromWall2 (){
        flywheelTask.setFlywheelPow(shootPower);
        driveTrain.moveLeftNInch(1, 8, 10, false);
        //pause();
        driveTrain.rotateDegreesPrecision(shootRotation2);
        //pause();
        sleep(500);
        driveTrain.moveBackwardNInch(0.3, 1, 3,false);
        driveTrain.moveBackwardNInch(0.8,14,3,false);
        driveTrain.moveBackwardNInch(0.3, 3, 3, false);
        sweeper.setPower(1);
        sleep(1500);
    }

    public void checkFirst() {
        driveTrain.moveBackwardNInch(0.4,42, 10, false);
        correctionStrafe();
        driveTrain.powerAllMotors(-0.15);
        boolean detectColor = false;
        boolean wrongColor = false;

        while (!detectColor && !wrongColor && opModeIsActive()) {
            if (timer.time() > 30) {
                detectColor = voiColorSensorTop.isBlue();
                if (voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue()){
                    driveTrain.stopAll();
                    wrongColor = true;
                }
                timer.reset();
            }
        }

        if (wrongColor && !detectColor){
            System.out.println("WRONG COLOR");
            correctionStrafe();
            pause();
            pushButton();
        } else {
            boolean blue = voiColorSensorTop.isBlue();
            boolean red = false;
            boolean white = false;
            System.out.println("BLUE2 " + blue);
            while (opModeIsActive() && blue && !white){
                if (timer.time()>30){
                    blue = voiColorSensorTop.isBlue();
                    timer.reset();
                    white = voiColorSensorBottom.isWhite();

                }
            }
            while (opModeIsActive() && !blue && !red && !white){
                if (timer.time()>30) {
                    blue = voiColorSensorTop.isBlue();
                    red = voiColorSensorTop.isRed() && !voiColorSensorTop.isBlue();
                    white = voiColorSensorBottom.isWhite();
                    timer.reset();
                }
            }
            if (red) {
                goBackAndPress();
                shootRotation = 90;
            }else if (white){
                driveTrain.moveForwardNInch(0.5, 2, 3, false);
            }
        }
    }

    public void coolDown() {
        timer.reset();
        while (opModeIsActive() && timer.time() < 1000);
        flywheelTask.setFlywheelPow(0.4);
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

    public void correctionStrafe() {
        driveTrain.moveRightNInch(0.2,5,0.5, false);
    }

    public void correctionStrafe(double seconds) {
        driveTrain.moveRightNInch(0.2, 5, seconds, false);
    }

    public void hitCapBall(){
        int initialDirection = imu.getAngle();
        driveTrain.moveBackwardNInch(1, 50, 10, true);
        driveTrain.rotateDegrees((int) (capBallRotation * 0.3), false);
        driveTrain.rotateDegrees((int)((initialDirection-imu.getAngle())*0.25), false);
        driveTrain.moveBackwardNInch(1, 18, 10, true);
    }

    public void hitCapBall2(){
        driveTrain.moveBackwardNInch(0.8, 36, 10, false);
    }

    public void pickUpBall(){
        //sweeper.setPower(1);
        sleep(1500);
        //sweeper.setPower(0);
        driveTrain.moveRightNInch(0.5, 10, 5, false);
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
