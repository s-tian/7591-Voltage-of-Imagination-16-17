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
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask.upPosition;

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
    public static Team team = Team.BLUE;
    // Options
    boolean missed = false,  detectRed1 = false, detectBlue1 = false;
    boolean missedLineUp = false;
    boolean shootFirst = false;
    boolean pickUp = true;

    boolean shFirst = true;

    final int topSensorID = 0x3a;
    final int bottomFrontID = 0x44;
    final int bottomBackID = 0x3c;

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
    int threeBallAngle = 50;

    double rShootRotation = 80;
    double rShootRotation2 = 135.5;
    double rSralt = 139.5;
    double rSralt3 = 95;

    double sFarRotB = 50; // shoot far rotation Blue near
    double sFarRotB2 = 48; // shoot far rotation Blue far
    double sFarRotR = 135; // shoot far rotation Red
    double sCloRotB = 108; // shoot close rotation Blue
    double sCloRotR = 80; // shoot close rotation Red

    // Powers
    double shootPower = 0.75; // normal shoot power
    double shootFirstPower = 0.58; // shoot first power
    double spalt = 0.6; // short shoot power (from first beacon)
    double spalt2 = .8; // shoot first (from starting position)
    double bpPower = 0.07; // beacon pressing driveTrain power

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

    DcMotor frontLeft, frontRight, backLeft, backRight;

    //Misc
    public static AutoMode autoMode = AutoMode.ThreeBall;
    public FlywheelTask flywheelTask;
    public IntakeTask intakeTask;
    MecanumDriveTrain driveTrain;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS), gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ButtonPusherTask buttonPusherTask;

    public enum AutoMode {
        TwoBall, ThreeBall, Defensive;

        @Override
        public String toString() {
            switch (this) {
                case TwoBall:
                    return "Two Ball";
                case ThreeBall:
                    return "Three Ball";
                case Defensive:
                    return "Defensive";
            }
            return "";
        }
    }

    public void runOpMode() {
        initialize();
        options();
        TaskThread.calculateVoltage(this);

        //telemetry.addData("Ready!", "");
        //telemetry.update();
        waitForStart();
        guide.setPosition(ButtonPusherTask.upPosition);
        gameTimer.reset();
        buttonPusherTask.start();
        intakeTask.start();

        if (team == Team.BLUE) {
            threeBallAngle = 43;
        }
        if (autoMode == AutoMode.ThreeBall) {
            pickUpBall();
            if (team == Team.BLUE) {
                lineUpToWall(38);
            } else {
                lineUpToWall(32);
            }
        } else if (autoMode == AutoMode.TwoBall){
            driveTrain.moveBackwardNInch(0.2, 15, 5, false, true);
            flywheelTask.setFlywheelPow(shootFirstPower);
            sleep(2000);
            sweeper.setPower(1);
            sleep(shootTime);
            sweeper.setPower(0);
            flywheelTask.setFlywheelPow(0);
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(threeBallAngle + wallAngle, 0.4);
            } else if (team == Team.RED) {
                driveTrain.rotateToAngle(-threeBallAngle + wallAngle, 0.4);
            }
            if (team == Team.BLUE) {
                lineUpToWall(38);
            } else {
                lineUpToWall(32);
            }
        } else {
            lineUpToWall(48);
        }
        dpb();
        drivePushButton2();
            if (missed) {
                checkFirst();
                moveFromWall();
            } else {
                if (autoMode != AutoMode.Defensive) {
                    moveFromWall2();
                    return;
                }
            }
        if (autoMode == AutoMode.Defensive) {
            defense();
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
        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        sweeper3 = hardwareMap.crservo.get("sweeper3");
        sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(topSensorID));
        voiColorSensorTop = new VOIColorSensor(colorSensorTop, this);
        button = hardwareMap.crservo.get("button");
        buttonPusherTask = new ButtonPusherTask(this);

        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        guide = hardwareMap.servo.get("guide");

        CapBallTask capBallTask = new CapBallTask(this);
        intakeTask = new IntakeTask(this);
        flywheelTask = new FlywheelTask(this);

        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,imu,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTask.start();
        wallAngle = imu.getAngle();
        guide.setPosition(ButtonPusherTask.upPosition);
        voiColorSensorTop.team = driveTrain.team = team;

    }

    public void pickUpBall(){
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(2);
        powerSweeper(1, 1250);
        sleep(1250);
        shootTime += 1000;

        if (shFirst) {
            if (team == Team.BLUE) {
                driveTrain.moveRightNInch(0.5,20, 10, false, true);
                //powerSweeper(-1, 200);
                driveTrain.rotateToAngle(wallAngle + 180, 0.4);
            } else if (team == Team.RED) {
                driveTrain.moveLeftNInch(0.5, 20, 10, false, true);
                //powerSweeper(-1, 200);
                driveTrain.rotateToAngle(wallAngle, 0.4);
            }
            timer.reset();
            flywheelTask.setFlywheelPow(shootFirstPower);
            while (flywheelTask.getFlywheelState() != FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET && opModeIsActive()) {
                telemetry.addData("Time", (int)gameTimer.time());
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.getFlywheelState());
                telemetry.update();
                if (timer.time() > 2000) {
                    // 5000 just for testing, change to 2000
                    break;
                }
            }
            sweeper.setPower(1);
            sleep(shootTime);
            coolDown();
        }
        if (team == Team.BLUE) {
            if (!shFirst) {
                driveTrain.moveRightNInch(0.5, 23, 5, false, true);
            }
            driveTrain.rotateToAngle(threeBallAngle + wallAngle, 0.4);
        } else if (team == Team.RED) {
            if (!shFirst) {
                driveTrain.moveLeftNInch(0.5, 23, 5, false, true);
            }
            driveTrain.rotateToAngle(-threeBallAngle + wallAngle, 0.4);
        }
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
        guide.setPosition(ButtonPusherTask.downPosition);
        int wlTimeout = 0; // timeout for white line detection (used for missing line)
        double fastDistance = distance - 0.5;

        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false);
        buttonPusherTask.extendButton = true;
        driveTrain.moveUpNInch(0.8, fastDistance, 10, false, true);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveRightNInch(1, 40, 10, true, true);
        //correctionStrafe();
        wallAngle = imu.getAngle();
    }

    public void dpb() {
        // Drive Push Button
        telemetry.addData("dpb", "");
        telemetry.update();
        int icto = 3000; // initial check timeout
        int scto = 3000; // secondary check timeout
        boolean behind = false;
        boolean pushCorrect = false;
        if (voiColorSensorTop.correctColor()) {
            // if detect blue then just push button
            correctionStrafe();
            pushButton();
            betweenBeacon += 3;
            return;
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
                    correctionStrafe();
                    if (team == Team.RED) {
                        frontRight.setPower(-0.5);
                        frontLeft.setPower(0.5);
                        sleep(300);
                    } else {
                        backLeft.setPower(-0.5);
                        backRight.setPower(0.5);
                        sleep(300);
                    }
                    pushButton();
                    return;
                }
            }
        }
        missed = true;
        betweenBeacon -= 8;
    }

    public void drivePushButton2() {
        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 8000;
        driveTrain.moveUpNInch(0.35, betweenBeacon - 8, 10, false, false);
        driveTrain.moveUpNInch(0.15, 8, 3, false, true);
        if (team == Team.RED) {
            frontRight.setPower(-0.5);
            frontLeft.setPower(0.5);
            sleep(300);
        } else {
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(300);
        }
        driveTrain.stopAll();

        driveTrain.powerUp(bpPower);
        boolean detectColor = false;
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
        driveTrain.stopAll();
        correctionStrafe();
        if (team == Team.RED) {
            frontRight.setPower(-0.5);
            frontLeft.setPower(0.5);
            sleep(300);
        } else {
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(300);
        }
        driveTrain.stopAll();
        wallAngle = imu.getAngle();
        pushButton();
        if (far) {
            sFarRotB = sFarRotB2;
        }
        if (!missed) {
            buttonPusherTask.withdrawButton = true;
            guide.setPosition(ButtonPusherTask.upPosition);
        }
    }

    public void moveFromWall(){
        telemetry.addData("moveFromWall", "");
        telemetry.update();
        wallAngle = imu.getAngle();
        driveTrain.moveLeftNInch(0.6, 6, 5, false, false);
        buttonPusherTask.withdrawButton = true;
        driveTrain.stopAll();
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sCloRotB);
        }
        if (team == Team.RED){
            driveTrain.rotateToAngle(wallAngle + sCloRotR);
        }
        if (shFirst) {
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
        guide.setPosition(ButtonPusherTask.upPosition);
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        telemetry.addData("moveFromWall2", "");
        telemetry.update();
        driveTrain.moveLeftNInch(0.6, 1, 10, false, true);
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        }
        if (team == Team.RED) {
            if (shFirst) {
                sFarRotR += 175;
            }
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }
        if (shootFirst || shFirst) {
            driveTrain.moveBackNInch(0.5, 30, 10, false, false);
            guide.setPosition(ButtonPusherTask.upPosition);
            driveTrain.moveBackNInch(0.5, 30, 10, false, true);

        } else {
            pause();
            driveTrain.moveBackwardNInch(0.3, 3, 3, false, false);
            driveTrain.moveBackwardNInch(0.8, 22, 10, false, false);
            driveTrain.moveBackwardNInch(0.3, 3, 3, false, false);
            sweeper.setPower(-0.3);
            timer2.reset();
            driveTrain.moveBackwardNInch(0.15, 2, 3, false, true);
            flywheelTask.setFlywheelPow(shootPower);
            guide.setPosition(upPosition);
            timer.reset();
            while (flywheelTask.getFlywheelState() != FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET && opModeIsActive()) {
                telemetry.addData("Time", (int)gameTimer.time());
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.getFlywheelState());
                telemetry.update();
                if (timer2.time() > 500) {
                    sweeper.setPower(0);
                }
                if (timer.time() > 5000) {
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
        shootPower = spalt;
        telemetry.update();
        driveTrain.moveBackNInch(0.3, betweenBeacon - 5, 10, false, false);
        driveTrain.moveBackNInch(0.15, 5, 10, false, true);
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

    public void pushButton(){
        driveTrain.stopAll();
        buttonPusherTask.pushButton = true;
        sleep(buttonPusherTask.pushTime + 200);
    }

    public void powerSweeper(double power, int time) {
        intakeTask.power = power;
        intakeTask.sweepTime = time;
    }

    public void defense() {
        if (team == Team.BLUE) {
            driveTrain.moveBackwardNInch(0.4, 2, 2, false, true);
            driveTrain.moveLeftNInch(1, 36, 10, true, true);
            driveTrain.moveForwardNInch(0.5,20, 5, false, true);
        } else if (team == Team.RED){
            driveTrain.moveLeftNInch(1, 43, 10, true, true);
            driveTrain.moveUpNInch(0.5, 15, 5, false, true);
        }
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        telemetry.addData("Time", gameTimer.time()*1.0/1000);
        telemetry.update();
        while (gameTimer.time() < 30000);

    }


    public void options(){
        boolean confirmed = false;
        while(!confirmed){

            // select team
            if (gamepad1.b){
                team = Team.RED;
            } else if (gamepad1.x){
                team = Team.BLUE;
            }

            // select mode
            if (gamepad1.dpad_left) {
                autoMode = AutoMode.TwoBall;
            } else if (gamepad1.dpad_up) {
                autoMode = AutoMode.ThreeBall;
            } else if (gamepad1.dpad_right) {
                autoMode = AutoMode.Defensive;
            }

            telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
            telemetry.addData("Mode", autoMode);
            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
                telemetry.addData("Mode", autoMode);
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                voiColorSensorTop.team = driveTrain.team = team;
                confirmed = true;
            }
        }
    }
}
