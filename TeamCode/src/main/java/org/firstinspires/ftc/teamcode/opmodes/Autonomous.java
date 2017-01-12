package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ViewerParameters;

import org.firstinspires.ftc.teamcode.Tests.ButtonTest;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;

import java.text.DecimalFormat;

/**
 * Created by Howard on 12/13/16.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Tests")

/*
 * The terms "up" and "back" are dependent upon team.
 * For blue team, up means driving forward and back means driving backwards.
 * For red team, up means driving backwards and back means driving forwards.
 * This is done so that "up" would always be travelling away from the start wall
 * and "back" will always be directed back towards the start wall.
 * "correct" color references the color of the team, while "badCol" references the opposing alliance's color
 */
public class Autonomous extends LinearOpMode {

    int delay = 200;
    Team team = Team.BLUE;
    // Options
    boolean missed = false,  detectRed1 = false, detectBlue1 = false;
    boolean missedLineUp = false;
    boolean shootFirst = false;
    boolean pickUp = true;

    final int topSensorID = 0x3a;
    final int bottomFrontID = 0x44;
    final int bottomBackID = 0x3c;

    double upPosition = 0.8;
    double downPosition = 0.1;

    double voltageLevel;
    double voltageFactor;
    int shootTime = 2500;

    double wld; // white line distance
    int betweenBeacon = 33; // far beacon distance
    int bbalt = 36; // near beacon distance
    int bbalt2 = 5; // miss first beacon distance

    // Angles
    final double pickUpRotB = 152; // pick up rotation Blue
    double shootRotation = 108; // first beacon shoot rotation near
    double shootRotation2 = 43
            ; // near shoot rotation
    double sralt = 38; // far shoot rotation
    double sralt3 = 90; // first beacons shoot rotation far
    double angle = -35; // normal white line rotation angle
    double sfAngle = -125; // shoot first alignment angle
    double angleAlt = -55; // shoot first rotation angle
    double wallAngle; // angle parallel to beacon wall

    double rShootRotation = 80;
    double rShootRotation2 = 135.5;
    double rSralt = 139.5;
    double rSralt3 = 95;

    double sFarRotB = 50; // shoot far rotation Blue near
    double sFarRotB2 = 46; // shoot far rotation Blue far
    double sFarRotR = 135; // shoot far rotation Red
    double sCloRotB = 108; // shoot close rotation Blue
    double sCloRotR = 80; // shoot close rotation Red

    // Powers
    double shootPower = 0.75; // normal shoot power
    double spalt = 0.6; // short shoot power (from first beacon)
    double spalt2 = .8; // shoot first (from starting position)
    double bpPower = 0.15; // beacon pressing driveTrain power

    double rShootPower = 0.69;
    double rSpalt = 0.6;
    double rSpalt2 = 0.8;

    // Hardware
    ColorSensor colorSensorTop, colorBottomFront, colorBottomBack;
    VOIColorSensor voiColorSensorTop, voiColorBottomBack, voiColorBottomFront;
    Servo forkLeft, forkRight, guide;
    CRServo button;
    VOIImu imu;
    VOISweeper sweeper;
    CRServo sweeper1, sweeper2, sweeper3;
    BNO055IMU adaImu;

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft; //sweeper, conveyor;

    //Misc
    public FlywheelTask flywheelTask;
    MecanumDriveTrain driveTrain;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS), buttonTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ButtonPusherTask buttonPusherTask;

    public void runOpMode() {
        initialize();
        //options();
        double mc7 = hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = hardwareMap.voltageSensor.get("flywheels").getVoltage();
        voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        flywheelTask.voltage = voltageLevel;
        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        buttonPusherTask.start();
        if (pickUp) {
            pickUpBall();
            lineUpToWall(25);
        } else if (shootFirst) {
            shoot();
            lineUpToWall(40);
        } else {
            lineUpToWall(48);
        }
        dpb();
        //drivePushButton();
        drivePushButton2();
        if (missed){
            //flywheelTask.setFlywheelPow(spalt);
            checkFirst();
            moveFromWall();
        }else {
            moveFromWall2();
        }
        flywheelTask.running = false;
        buttonPusherTask.running = false;
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
        sweeper3 = hardwareMap.crservo.get("sweeper3");
        sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorBottomBack = hardwareMap.colorSensor.get("colorBottomBack");
        colorBottomFront = hardwareMap.colorSensor.get("colorBottomFront");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(topSensorID));//maybe create8bit
        colorBottomFront.setI2cAddress(I2cAddr.create8bit(bottomFrontID));
        colorBottomBack.setI2cAddress(I2cAddr.create8bit(bottomBackID));
        voiColorSensorTop = new VOIColorSensor(colorSensorTop, this);
        voiColorBottomFront = new VOIColorSensor(colorBottomFront, this);
        voiColorBottomBack = new VOIColorSensor(colorBottomBack, this);
        button = hardwareMap.crservo.get("button");
        buttonPusherTask = new ButtonPusherTask(this, button);

        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        guide = hardwareMap.servo.get("guide");
        CapBallTask capBallTask = new CapBallTask(this); // for forklift initialization
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.start();
        wallAngle = imu.getAngle();
        guide.setPosition(downPosition);
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

    public void lineUpToWall(double distance) {
        /*
        The purpose of this method is to have the robot align with the wall, so that it can move along it while detecting beacons.
        This method is used for driving towards the white line until the bottom color sensor detects the white line.
        Because of the position of the button pusher, the robot on the red alliance must move backwards.
        The steps of this method include:
            1. Drive to the white line (forwards for blue, backwards for red)
            2. Rotate so that robot is parallel with wall.
            3. Strafe towards the robot with considerable power to ensure lining up correctly.
         */
        telemetry.addData("lineUpToWall", "");
        telemetry.update();
        int wlTimeout = 0; // timeout for white line detection (used for missing line)
        double fastDistance = distance - 0.5;

        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false);
        buttonPusherTask.extendButton = true;
        driveTrain.moveUpNInch(0.8, fastDistance, 10, false, true);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveRightNInch(0.75, 40, 10, true, true);
        //correctionStrafe();
        wallAngle = imu.getAngle();
    }

    public void dpb() {
        // Drive Push Button
        telemetry.addData("dpb", "");
        telemetry.update();
        int icto = 2000; // initial check timeout
        int scto = 3000; // secondary check timeout
        boolean behind = false;
//        if (team == Team.RED) {
//            boolean pushRed = false;
//            if (voiColorSensorTop.isRed()) {
//                // if detect blue then just push button
//                pushButton();
//                pushRed = true;
//            } else if (voiColorSensorTop.isBlue()) {
//                firstBlue();
//            } else {
//                // if no color detected, first move backwards to check for blue
//                driveTrain.powerAllMotors(bpPower);
//                timer.reset();
//                while (opModeIsActive() && timer.time() < icto) {
//                    if (voiColorSensorTop.isRed()) {
//                        driveTrain.stopAll();
//                        correctionStrafe();
//                        pushButton();
//                        pushRed = true;
//                        break;
//                    } else if (voiColorSensorTop.isBlue()) {
//                        firstBlue();
//                        pushRed = true;
//                        break;
//                    }
//                    if (VOIImu.subtractAngles(imu.getAngle(), wallAngle) <= -2) {
//                        behind = true;
//                        driveTrain.moveBackwardNInch(0.2,1.5, 3, false, true);
//                        driveTrain.rotateDegreesPrecision(3);
//                        break;
//                    }
//                }
//
//                if (!pushRed && (timer.time() >= icto || behind)) {
//                    boolean addedTime = false;
//                    driveTrain.powerAllMotors(-bpPower);
//                    timer.reset();
//                    while (opModeIsActive() && timer.time() < scto) {
//                        if (voiColorSensorTop.isRed()) {
//                            driveTrain.stopAll();
//                            correctionStrafe();
//                            pushButton();
//                            pushRed = true;
//                            break;
//                        }
//                        else if (voiColorSensorTop.isBlue() && !addedTime) {
//                            scto += 1000;
//                            addedTime = true;
//                        }
//                    }
//                    if (timer.time() >= scto && !pushRed) {
//                        correctionStrafe();
//                        missed = true;
//                        betweenBeacon = bbalt2;
//                    }
//                }
//            }
//        } else {
//            boolean pushBlue = false;
//            if (voiColorSensorTop.isBlue()) {
//                // if detect blue then just push button
//                pushButton();
//                pushBlue = true;
//            } else if (voiColorSensorTop.isRed()) {
//                firstRed();
//            } else {
//                // if no color detected, first move backwards to check for blue
//                driveTrain.powerAllMotors(-bpPower);
//                timer.reset();
//                while (opModeIsActive() && timer.time() < icto) {
//
//                    if (voiColorSensorTop.isBlue()) {
//                        driveTrain.stopAll();
//                        correctionStrafe();
//                        pushButton();
//                        pushBlue = true;
//                        break;
//                    } else if (voiColorSensorTop.isRed()) {
//                        firstRed();
//                        pushBlue = true;
//                        break;
//                    }
//                    if (VOIImu.subtractAngles(imu.getAngle(), wallAngle) <= -2) {
//                        behind = true;
//                        driveTrain.moveForwardNInch(0.2, 1.5, 3, false, true);
//                        driveTrain.rotateDegreesPrecision(3);
//                        break;
//                    }
//                }
//
//                if (!pushBlue && (timer.time() >= icto || behind)) {
//                    boolean addedTime = false;
//                    driveTrain.powerAllMotors(bpPower);
//                    timer.reset();
//                    while (opModeIsActive() && timer.time() < scto) {
//                        if (voiColorSensorTop.isBlue()) {
//                            driveTrain.stopAll();
//                            correctionStrafe();
//                            pushButton();
//                            pushBlue = true;
//                            break;
//                        } else if (voiColorSensorTop.isRed() && !addedTime) {
//                            scto += 1000;
//                            addedTime = true;
//                        }
//                    }
//                    if (timer.time() >= scto && !pushBlue) {
//                        correctionStrafe();
//                        missed = true;
//                        betweenBeacon = bbalt2;
//                    }
//                }
//            }
//        }
        boolean pushCorrect = false;
        if (voiColorSensorTop.correctColor()) {
            // if detect blue then just push button
            pushButton();
            betweenBeacon += 3;
//        } else if (voiColorSensorTop.wrongColor()) {
//            // use algorithm for detecting wrong color immediately
//            firstWrong();
//        } else {
//            // if no color detected, first move back to check for blue
//            driveTrain.powerUp(-bpPower);
//            timer.reset();
//            while (opModeIsActive() && timer.time() < icto) {
//                if (voiColorSensorTop.correctColor()) {
//                    driveTrain.stopAll();
//                    correctionStrafe();
//                    pushButton();
//                    pushCorrect = true;
//                    break;
//                } else if (voiColorSensorTop.wrongColor()) {
//                    firstWrong();
//                    pushCorrect = true;
//                    break;
//                }
//                if (VOIImu.subtractAngles(imu.getAngle(), wallAngle) <= -2) {
//                    behind = true;
//                    driveTrain.moveUpNInch(0.2,1.5, 3, false, true);
//                    driveTrain.rotateDegreesPrecision(3);
//                    break;
//                }
//            }
//            // drive up until correct color detected
//            if (!pushCorrect && (timer.time() >= icto || behind)) {
//                boolean addedTime = false;
//                driveTrain.powerUp(bpPower);
//                timer.reset();
//                while (opModeIsActive() && timer.time() < scto) {
//                    if (voiColorSensorTop.correctColor()) {
//                        driveTrain.stopAll();
//                        correctionStrafe();
//                        pushButton();
//                        pushCorrect = true;
//                        break;
//                    }
//                    else if (voiColorSensorTop.wrongColor() && !addedTime) {
//                        scto += 1000;
//                        addedTime = true;
//                    }
//                }
//                if (timer.time() >= scto && !pushCorrect) {
//                    correctionStrafe();
//                    missed = true;
//                    betweenBeacon = bbalt2;
//                }
//            }
        } else {
            timer.reset();
            boolean timeAdded = false;
            driveTrain.powerUp(bpPower);
            while (timer.time() < icto & opModeIsActive()) {
                if (voiColorSensorTop.wrongColor() && !timeAdded) {
                    icto += 1000;
                    timeAdded = true;
                }
                if (voiColorSensorTop.correctColor()) {
                    pushButton();
                    break;
                }
            }
        }
    }

    public void drivePushButton2() {

        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 5000;
        driveTrain.moveUpNInch(0.8, betweenBeacon, 10, false, true);
        correctionStrafe();
        wallAngle = imu.getAngle();
        boolean detectColor = false;
        driveTrain.powerUp(bpPower);
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        boolean far = false;
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // determine which side of beacon
                if (voiColorSensorTop.wrongColor()) {
                    //rShootRotation2 = rSralt;
                    far = true;
                    timeo += 1000;
                }
                detectColor = voiColorSensorTop.correctColor();
                timer.reset();
            }
            if (timeout.time() >= timeo) {
                giveUpSecond();
            }
        }
        correctionStrafe();
        pushButton();
//        if (far) {
//            driveTrain.powerUp(-0.2);
//        } else {
//            driveTrain.powerUp(0.2);
//        }
//        timer.reset();
//        while (opModeIsActive() && timer.time() < 1500) {
//            if (voiColorBottomBack.isWhite()) {
//                break;
//            }
//        }
//        driveTrain.stopAll();
        if (far) {
            sFarRotB = sFarRotB2;
        }
        buttonPusherTask.withdrawButton = true;
    }

    public void moveFromWall(){
        telemetry.addData("moveFromWall", "");
        telemetry.update();
        wallAngle = imu.getAngle();
//        if (team == Team.RED) {
//            driveTrain.moveLeftNInch(0.6, 6, 10, false, true);
//            //double offset = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
//            //rShootRotation = VOIImu.subtractAngles(rShootRotation, offset, false);
//            flywheelTask.setFlywheelPow(rShootPower);
//            driveTrain.rotateDegreesPrecision(rShootRotation);
//            if (shootFirst) {
//                hitCapBall();
//            } else {
//                sleep(2000);
//                sweeper.setPower(1);
//                sleep(shootTime);
//                coolDown();
//            }
//            //parkSide();
//        } else {
//            driveTrain.moveLeftNInch(0.6, 6, 10, false, true);
//            double offset = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
//
//            shootRotation = VOIImu.subtractAngles(shootRotation, offset);
//
//            driveTrain.rotateDegreesPrecision(shootRotation);
//            if (shootFirst) {
//                hitCapBall();
//            } else {
//                sleep(200);
//                flywheelTask.setFlywheelPow(shootPower);
//                sleep(2000);
//                sweeper.setPower(1);
//                sleep(shootTime);
//                coolDown();
//            }
//            //parkSide();
//        }
        driveTrain.moveLeftNInch(0.6, 6, 5, false, false);
        buttonPusherTask.withdrawButton = true;
        driveTrain.stopAll();
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sCloRotB);
        }
        if (team == Team.RED){
            driveTrain.rotateToAngle(wallAngle + sCloRotR);
        }
        if (shootFirst) {
            hitCapBall();
        } else {
            sleep(200);
            flywheelTask.setFlywheelPow(shootPower);
            sleep(2000);
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
        }
    }

    public void moveFromWall2(){
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        telemetry.addData("moveFromWall2", "");
        telemetry.update();
        driveTrain.moveLeftNInch(0.6, 1, 10, false, true);
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        }
        if (team == Team.RED) {
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }
        if (shootFirst) {
            driveTrain.moveBackwardNInch(0.5, 50, 10, false, true);
        } else {
            pause();
            driveTrain.moveBackwardNInch(0.3, 3, 3, false, false);
            driveTrain.moveBackwardNInch(0.8, 22, 10, false, false);
            driveTrain.moveBackwardNInch(0.3, 3, 3, false, false);
            driveTrain.moveBackwardNInch(0.15, 2, 3, false, true);
            flywheelTask.setFlywheelPow(shootPower);
            guide.setPosition(upPosition);
            timer.reset();
            while (flywheelTask.getFlywheelState() != FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET && opModeIsActive()) {
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.getFlywheelState());
                telemetry.update();
                if (timer.time() > 1500) {
                    // 5000 just for testing, change to 2000
                    break;
                }
            }
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
            hitCapBall2();
        }
    }

    public void checkFirst() {
        telemetry.addData("checkFirst", "");
//        if (team == Team.RED) {
//            driveTrain.moveForwardNInch(0.4,42, 10, false, true);
//            correctionStrafe();
//            driveTrain.powerAllMotors(0.15);
//            boolean isBlue = false;
//            boolean rammedBlue = false;
//            while (opModeIsActive() && !voiColorSensorTop.isRed() && !rammedBlue) {
//                if (voiColorSensorTop.isRed() && !isBlue) {
//                    isBlue = true;
//                    timer.reset();
//                }
//                if (isBlue && timer.time() > 1000) {
//                    rammedBlue = true;
//                    driveTrain.stopAll();
//                }
//            }
//            if (rammedBlue) {
//                correctionStrafe();
//                driveTrain.powerAllMotors(-0.1);
//                while(opModeIsActive() && !voiColorSensorTop.isRed()) {
//                }
//                driveTrain.stopAll();
//                correctionStrafe();
//                pushButton();
//            }
//            else {
//                driveTrain.stopAll();
//                correctionStrafe();
//                pushButton();
//            }
//            if (!isBlue) {
//                rShootRotation = rSralt3;
//            }
//        } else {
//            shootPower = spalt;
//            telemetry.update();
//            driveTrain.moveBackwardNInch(0.4, 42, 10, false, true);
//            correctionStrafe();
//            driveTrain.powerAllMotors(-0.15);
//            boolean isRed = false;
//            boolean rammedRed = false;
//            while (opModeIsActive() && !voiColorSensorTop.isBlue() && !rammedRed) {
//                if (voiColorSensorTop.isRed() && !isRed) {
//                    isRed = true;
//                    timer.reset();
//                }
//                if (isRed && timer.time() > 1000) {
//                    rammedRed = true;
//                    driveTrain.stopAll();
//                }
//            }
//            if (rammedRed) {
//                correctionStrafe();
//                driveTrain.powerAllMotors(0.1);
//                while (opModeIsActive() && !voiColorSensorTop.isRed()) {
//                }
//                driveTrain.stopAll();
//                correctionStrafe();
//                pushButton();
//            } else {
//                driveTrain.stopAll();
//                correctionStrafe();
//                pushButton();
//            }
//            if (!isRed) {
//                shootRotation = sralt3;
//            }
//        }
        shootPower = spalt;
        telemetry.update();
        driveTrain.moveBackNInch(0.4, 42, 10, false, true);
        correctionStrafe();
        driveTrain.powerUp(-0.15);
        boolean isWrong = false;
        boolean rammedWrong = false;
        while (opModeIsActive() && !voiColorSensorTop.correctColor() && !rammedWrong) {
            if (voiColorSensorTop.wrongColor() && !isWrong) {
                isWrong = true;
                timer.reset();
            }
            if (isWrong && timer.time() > 1000) {
                rammedWrong = true;
                driveTrain.stopAll();
            }
        }
        if (rammedWrong) {
            correctionStrafe();
            driveTrain.powerUp(bpPower);
            while (opModeIsActive() && !voiColorSensorTop.wrongColor()) {
            }
            driveTrain.stopAll();
            correctionStrafe();
            pushButton();
        } else {
            driveTrain.stopAll();
            correctionStrafe();
            pushButton();
        }
        if (!isWrong) {
            shootRotation = sralt3;
        }

    }

    public void coolDown() {
        sweeper.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void giveUpSecond() {
//        if (team == Team.RED) {
//            telemetry.addData("giveUpSecond", "");
//            telemetry.update();
//            driveTrain.moveForwardNInch(0.3, 35, 10, false, true);
//            correctionStrafe(1);
//            driveTrain.powerAllMotors(0.2);
//            timer.reset();
//            while (opModeIsActive() /*&& !voiColorSensorBottom.isWhite() */&& timer.time() < 10000) {
//            }
//            if (timer.time() >= 10000) {
//                stop();
//            }
//            correctionStrafe();
//            moveFromWall();
//            stop();
//        } else {
//            telemetry.addData("giveUpSecond", "");
//            telemetry.update();
//            driveTrain.moveBackwardNInch(0.3, 35, 10, false, true);
//            correctionStrafe(1);
//            driveTrain.powerAllMotors(-0.2);
//            timer.reset();
//            while (opModeIsActive() && /*!voiColorSensorBottom.isWhite() && */ timer.time() < 10000) {
//
//            }
//            if (timer.time() >= 10000) {
//                stop();
//            }
//            correctionStrafe();
//            moveFromWall();
//            stop();
//        }
        telemetry.addData("giveUpSecond", "");
        telemetry.update();
        driveTrain.moveBackNInch(0.3, 35, 10, false, true);
        correctionStrafe(1);
        driveTrain.powerUp(-0.2);
        timer.reset();
        while (opModeIsActive() && timer.time() < 10000) {
            if (voiColorBottomBack.isWhite() || voiColorBottomFront.isWhite()) {
                break;
            }
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
        telemetry.addData("hitCapBall", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(1, 50, 10, true, true);
        driveTrain.rotateDegrees(-270, 1, false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true);
    }

    public void hitCapBall2() {
        telemetry.addData("hitCapBall2", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(0.4, 30, 10, false, true);

    }

    public void pickUpBall(){
        sweeper.setPower(1);
        flywheelTask.setFlywheelPow(-0.4);
        sleep(1500);
        sweeper.setPower(0);
        shootTime += 1000;
        flywheelTask.setFlywheelPow(0);
        if (team == Team.BLUE) {
            driveTrain.moveRightNInch(0.5, 20, 5, false, true);
            driveTrain.rotateToAngle(60);
        } else if (team == Team.RED) {
            driveTrain.moveLeftNInch(0.5, 20, 5, false, true);
            driveTrain.rotateToAngle(-60);
        }
    }

    public void pushButton(){
        driveTrain.stopAll();
        buttonPusherTask.pushButton = true;
        sleep(buttonPusherTask.pushTime);
    }

    public void options(){
        telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
        telemetry.update();
        boolean confirmed = false;
        while(!confirmed){
            if (gamepad1.a){
                team = Team.RED;
            }
            if (gamepad1.b){
                team = Team.BLUE;
            }
            telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                voiColorSensorTop.team = driveTrain.team = team;
                confirmed = true;
            }
        }
    }
}
