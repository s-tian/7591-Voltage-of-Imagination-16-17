package org.firstinspires.ftc.teamcode.opmodes;

/**
 * Created by Howard on 12/10/16.
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
    int delay = 200;

    // Options
    boolean missed = false,  detectBlue1 = false;
    boolean missedLineUp = false;
    boolean shootFirst = false;
    boolean defense = false;

    final int topSensorID = 0x3c;
    final int bottomSensorID = 0x44;

    double voltageLevel;
    int shootTime = 2500;

    // Distances
    int betweenBeacon = 30; // far beacon distance
    int bbalt = 34; // near beacon distance
    int bbalt2 = 10; // miss first beacon distance

    // Angles
    final double pickUpRotation = 152;
    double shootRotation = 80; // first beacon shoot rotation near
    double shootRotation2 = 135.5; // near shoot rotation
    double sralt = 139.5; // far shoot rotation
    double sralt3 = 95; // first beacons shoot rotation far
    double parallelAngle; // angle perpendicular to start wall (should be approx equal to wallAngle)
    double angle = 35; // normal white line rotation angle
    double sfAngle = 55; // shoot first alignment angle
    double angleAlt = -55; // shoot first rotation angle
    double wallAngle; // angle parallel to beacon wall

    // Powers
    double shootPower = 0.69; // normal shoot power
    double spalt = 0.6; // short shoot power (from first beacon)
    double spalt2 = .8; // shoot first (from starting position)
    double bpPower = 0.11; // beacon pressing driveTrain power

    // Hardware
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
        initialize();
        //options();
        double mc7 = hardwareMap.voltageSensor.get("Motor Controller 7").getVoltage();
        double mc6 = hardwareMap.voltageSensor.get("Motor Controller 6").getVoltage();
        double mc3 = hardwareMap.voltageSensor.get("Motor Controller 3").getVoltage();
        double mc2 = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        flywheelTask.voltage = voltageLevel;
        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        if (shootFirst) {
            shoot();
            lineUpToWall(35);
        } else {
            lineUpToWall(48);
        }
        dpb();
        //drivePushButton();
        drivePushButton2();
        if (missed){
            checkFirst();
            moveFromWall();
        }else {
            if (defense) {
                trollGS();
            } else {
                moveFromWall2();
            }
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
        button.setPosition(0);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.start();
        parallelAngle = VOIImu.addAngles(imu.getAngle(), angle);

    }

    public void shoot() {
        flywheelTask.setFlywheelPow(spalt2);
        driveTrain.moveBackwardNInch(0.2, 12, 5, false, true);
        angle = angleAlt;
        sleep(1000);
        sweeper.setPower(1);
        sleep(1500);
        coolDown();
        driveTrain.moveBackwardNInch(0.2, 5, 5, false, true);
        driveTrain.rotateDegreesPrecision(sfAngle);
    }

    public void lineUpToWall(int distance) {
        telemetry.addData("lineUpToWall", "");
        telemetry.update();
        int wlTimeout = 700; // timeout for white line detection (used for missing line)
        driveTrain.moveBackwardNInch(0.2, 0.5, 10, false, false);
        driveTrain.moveBackwardNInch(0.5, distance-0.5, 10, false, false);
        //pause();
        driveTrain.powerAllMotors(-0.15);
        boolean detectColor = false;
        timer.reset();
        timer2.reset();
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 30) {
                if (voiColorSensorBottom.isWhite()) {
                    detectColor = true;
                }
                timer.reset();
                if (timer2.time() > wlTimeout){
                    driveTrain.stopAll();
                    break;
                }
            }
        }
        //if (detectColor) {
        driveTrain.rotateDegreesPrecision(angle);
//        } else {
//            missedLineUp = true;
//            driveTrain.moveBackwardNInch(0.2, 3, 3, false, true);
//            int rotationAngle = VOIImu.subtractAngles(parallelAngle, imu.getAngle(), false);
//            driveTrain.rotateDegreesPrecision(rotationAngle);
//        }

        // ram into wall to straighten out
        driveTrain.moveRightNInch(0.75, 40, 10, true, true);
        correctionStrafe();
        wallAngle = imu.getAngle();

    }

    public void drivePushButton() {

        // move backwards to get behind beacon
        if (!missedLineUp) {
            driveTrain.moveBackwardNInch(0.2, 6, 10, false, true);
        }
        // move forward until beacon detected
        pause();
        driveTrain.powerAllMotors(0.1);
        boolean detectColor = false;
        int initialTicks = backRight.getCurrentPosition();
        timer.reset();
        timer2.reset();
        missed = false;
        while (!detectColor && opModeIsActive()) {
            if (timer.time() > 15) {
                detectColor = voiColorSensorTop.isBlue();
                timer.reset();
                if (voiColorSensorTop.isRed() && !detectBlue1) {
                    detectBlue1 = true;
                    initialTicks = backRight.getCurrentPosition();
                }

                if ((detectBlue1 && backRight.getCurrentPosition()-initialTicks > 7*driveTrain.TICKS_PER_INCH_FORWARD) || timer2.time()>5000) {
                    //pause();
                    betweenBeacon = bbalt2;
                    missed = true;
                    return;
                }
            }
        }

        if (!detectBlue1)
            betweenBeacon = bbalt;
        driveTrain.stopAll();
        //pause();
        // move forward to align button pusher with beacon button and push
        //pause();
        correctionStrafe();
        pause();
        pushButton();
    }

    public void dpb() {
        telemetry.addData("dpb", "");
        telemetry.update();
        int icto = 1000; // initial check timeout
        int scto = 3000; // secondary check timeout
        boolean behind = false;
        boolean pushRed = false;
        if (voiColorSensorTop.isRed()) {
            // if detect blue then just push button
            pushButton();
            pushRed = true;
        } else if (voiColorSensorTop.isBlue()) {
            firstBlue();
        } else {
            // if no color detected, first move backwards to check for blue
            driveTrain.powerAllMotors(bpPower);
            timer.reset();
            while (opModeIsActive() && timer.time() < icto) {
                if (voiColorSensorTop.isRed()) {
                    driveTrain.stopAll();
                    correctionStrafe();
                    pushButton();
                    pushRed = true;
                    break;
                } else if (voiColorSensorTop.isBlue()) {
                    firstBlue();
                    pushRed = true;
                    break;
                }
                if (VOIImu.subtractAngles(imu.getAngle(), wallAngle) <= -2) {
                    behind = true;
                    driveTrain.moveBackwardNInch(0.2,1.5, 3, false, true);
                    driveTrain.rotateDegreesPrecision(3);
                    break;
                }
            }

            if (!pushRed && (timer.time() >= icto || behind)) {
                boolean addedTime = false;
                driveTrain.powerAllMotors(-bpPower);
                timer.reset();
                while (opModeIsActive() && timer.time() < scto) {
                    if (voiColorSensorTop.isRed()) {
                        driveTrain.stopAll();
                        correctionStrafe();
                        pushButton();
                        pushRed = true;
                        break;
                    }
                    else if (voiColorSensorTop.isBlue() && !addedTime) {
                        scto += 1000;
                        addedTime = true;
                    }
                }
                if (timer.time() >= scto && !pushRed) {
                    correctionStrafe();
                    missed = true;
                    betweenBeacon = bbalt2;
                }
            }
        }
    }

    public void firstBlue() {
        telemetry.addData("firstBlue", "");
        telemetry.update();
        /*
        The algorithm must work for detecting blue on both sides of the beacon.
        I go backwards until I don't sense red anymore.
        Now, there are 2 situations that the robot may be in:
            1. Behind the beacon entirely
            2. Between the red and blue parts of the beacon
        In order to test for condition 2, I must go backwards for a certain length of time while looking for blue.
        However, if the time has passed and blue is still not detected, I can assume that the situation is condition 1.
         */
        int timeout = 600; // This timeout is the "certain length of time" referenced above.
        driveTrain.powerAllMotors(bpPower);
        boolean pushedRed = false;
        while(opModeIsActive() && voiColorSensorTop.isBlue()) {
        }
        timer.reset();
        while (opModeIsActive() && timer.time() < timeout) {
            if (voiColorSensorTop.isRed()) {
                driveTrain.stopAll();
                betweenBeacon = bbalt;
                correctionStrafe();
                pushButton();
                pushedRed = true;
                break;
            }
        }
        // if blue is not behind red, then drive forward until blue detected
        if (!pushedRed && timer.time() >= timeout) {
            correctionStrafe();
            driveTrain.powerAllMotors(-0.15);
            while (opModeIsActive() && !voiColorSensorTop.isRed());
            driveTrain.stopAll();
            correctionStrafe();
            pushedRed = true;
            pushButton();
        }
    }
    public void drivePushButton2() {
        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 5000;
        driveTrain.moveBackwardNInch(0.45, betweenBeacon, 10, false, false);
        //pause();
        correctionStrafe(1);
        wallAngle = imu.getAngle();
        boolean detectColor = false;
        driveTrain.powerAllMotors(-bpPower);
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // determine which side of beacon
                if (voiColorSensorTop.isBlue()){
                    shootRotation2 = sralt;
                    timeo += 1000;
                }
                detectColor = voiColorSensorTop.isRed();
                timer.reset();
            }
            if (timeout.time() >= timeo) {
                giveUpSecond();
            }
        }
        correctionStrafe(0.5);
        pushButton();
    }

    public void moveFromWall(){
        telemetry.addData("moveFromWall", "");
        telemetry.update();
        wallAngle = imu.getAngle();
        driveTrain.moveLeftNInch(0.6, 6, 10, false, true);
        //double offset = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
        //shootRotation = VOIImu.subtractAngles(shootRotation, offset, false);
        flywheelTask.setFlywheelPow(shootPower);
        driveTrain.rotateDegreesPrecision(shootRotation);
        if (shootFirst) {
            hitCapBall();
        } else {
            sleep(2000);
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
        }
        //parkSide();
    }

    public void moveFromWall2(){
        telemetry.addData("moveFromWall2", "");
        telemetry.update();
        driveTrain.moveLeftNInch(0.6, 3, 10, false, true);
        //double offset = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
        //shootRotation2 = VOIImu.subtractAngles(shootRotation2, offset, false);
        driveTrain.rotateDegreesPrecision(shootRotation2);
        if (shootFirst) {
            driveTrain.moveBackwardNInch(0.5, 50, 10, false, true);
        }
        else {
            sleep(500);
            driveTrain.moveBackwardNInch(0.2, 1, 3, false, false);
            driveTrain.moveBackwardNInch(0.3, 18, 3, false, false);
            driveTrain.moveBackwardNInch(0.15, 5, 3, false, true);
            sleep(250);
            flywheelTask.setFlywheelPow(shootPower);
            sleep(2000);
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
            hitCapBall2();
        }
    }

    public void checkFirst() {
        telemetry.addData("checkFirst", "");
        telemetry.update();
        driveTrain.moveForwardNInch(0.4,42, 10, false, true);
        correctionStrafe();
        driveTrain.powerAllMotors(0.15);
        boolean isBlue = false;
        boolean rammedBlue = false;
        while (opModeIsActive() && !voiColorSensorTop.isRed() && !rammedBlue) {
            if (voiColorSensorTop.isRed() && !isBlue) {
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
            while(opModeIsActive() && !voiColorSensorTop.isRed()) {
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
        if (!isBlue) {
            shootRotation = sralt3;
        }
    }

    public void coolDown() {
        sweeper.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void giveUpSecond() {
        telemetry.addData("giveUpSecond", "");
        telemetry.update();
        driveTrain.moveForwardNInch(0.3, 35, 10, false, true);
        correctionStrafe(1);
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

    public void trollGS() {
        driveTrain.moveLeftNInch(1,35,10, false, true);
        sleep(1000);
        driveTrain.moveBackwardNInch(0.5, 15, 10, false, true);
    }

    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        driveTrain.moveRightNInch(0.2, 5, seconds, false, true);
    }

    public void hitCapBall(){
        telemetry.addData("hitCapBall", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(1, 50, 10, true, true);
        driveTrain.rotateDegrees(-270, 1, false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true);
    }

    public void parkSide() {
        telemetry.addData("parkSide", "");
        telemetry.update();
        //driveTrain.moveForwardNInch(0.3, 4, 5, false, true);
        double rotation = VOIImu.subtractAngles(wallAngle, imu.getAngle(), false);
        driveTrain.rotateDegrees(rotation, 0.8, true);
        driveTrain.moveForwardNInch(1, 25, 5 , false, true);
    }

    public void hitCapBall2(){
        telemetry.addData("hitCapBall2", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(0.4, 45, 10, false, true);
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
        //telemetry.addData("Shoot first?", shootFirst ? "Yes" : "No");
        telemetry.addData("Defense?", defense ? "Yes" : "No");
        telemetry.update();
        boolean confirmed = false;
        while(!confirmed){
            if (gamepad1.a){
                //shootFirst = true;
                defense = true;
            }
            if (gamepad1.b){
                //shootFirst = false;
                defense = false;
            }
            //telemetry.addData("Shoot first?", shootFirst ? "Yes" : "No");
            telemetry.addData("Defense?", defense ? "Yes" : "No");

            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                //telemetry.addData("Shoot first?", shootFirst ? "Yes" : "No");
                telemetry.addData("Defense?", defense ? "Yes" : "No");
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                confirmed = true;
            }

        }
    }

}

