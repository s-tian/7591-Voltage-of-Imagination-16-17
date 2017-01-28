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

/**
 * The terms "up" and "back" are dependent upon team.
 * For blue team, up means driving forward and back means driving backwards.
 * For red team, up means driving backwards and back means driving forwards.
 * This is done so that "up" would always be travelling away from the start wall
 * and "back" will always be directed back towards the start wall.
 * "correct" color references the color of the team, while "wrong" references the opposing
 * alliance's color
 */
public class Autonomous extends LinearOpMode {

    int delay = 200;
    public static Team team = Team.BLUE;
    // Options
    boolean missed = false;
    final int topSensorID = 0x3a;

    int shootTime = 2500;

    int betweenBeacon = 33; // far beacon distance

    boolean shootFirst = false;
    // Angles
    double shootRotation = 108; // first beacon shoot rotation near
    double sralt3 = 90; // first beacons shoot rotation far
    double wallAngle; // angle parallel to beacon wall
    int threeBallAngle = 50;

    double sFarRotB = 33; // shoot far rotation Blue near
    double sFarRotB2 = 33; // shoot far rotation Blue far
    double sFarRotR = 135; // shoot far rotation Red
    double sCloRotB = 108; // shoot close rotation Blue
    double sCloRotR = 80; // shoot close rotation Red

    // Powers
    double shootPower = 0.75; // normal shoot power
    double shootFirstPower = 1; // shoot first power
    double spalt = 0.6; // short shoot power (from first beacon)
    double bpPower = 0.06; // beacon pressing driveTrain power

    // Hardware
    ColorSensor colorSensorTop;
    VOIColorSensor voiColorSensorTop;
    Servo guide;
    BNO055IMU adaImu;
    VOIImu imu;

    DcMotor frontLeft, frontRight, backLeft, backRight;

    //Misc
    public static AutoMode autoMode = AutoMode.ThreeBall;
    public static ParkMode parkMode = ParkMode.Corner;
    FlywheelTask flywheelTask;
    IntakeTask intakeTask;
    ButtonPusherTask buttonPusherTask;

    DecimalFormat df = new DecimalFormat();

    public static final AutoMode[] modes = {AutoMode.ThreeBall, AutoMode.TwoBall, AutoMode.Defensive};

    MecanumDriveTrain driveTrain;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum AutoMode {
        TwoBall, ThreeBall, Defensive;
    }

    public enum ParkMode {
        Corner, Center
    }

    @Override
    public void runOpMode() {
        initialize();
        options();

        //telemetry.addData("Ready!", "");
        //telemetry.update();
        waitForStart();
        System.out.println("Autonomous started!");
        guide.setPosition(ButtonPusherTask.upPosition);
        gameTimer.reset();
        flywheelTask.start();
        buttonPusherTask.start();
        intakeTask.start();

        if (team == Team.BLUE) {
            threeBallAngle = 43;
        }
        if (autoMode == AutoMode.ThreeBall) {
            pickUpBall();
            if (shootFirst) {
                shootFirstLineUp();
            } else {
                lineUpToWall(32);
            }
        } else if (autoMode == AutoMode.TwoBall){
            shootTwo();
            if (shootFirst) {
                shootFirstLineUp();
            } else {
                lineUpToWall(32);
            }
        } else {
            lineUpToWall(40);
        }
        drivePushButton();
        drivePushButton2();
            if (missed) {
                checkFirst();
                moveFromWall();
            } else {
                if (autoMode != AutoMode.Defensive) {
                    if (parkMode == ParkMode.Center) {
                        moveFromWall2();
                    } else if (parkMode == ParkMode.Corner) {
                        parkCorner();
                    }
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

        // Initialize color sensor
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(topSensorID));
        voiColorSensorTop = new VOIColorSensor(colorSensorTop, this);

        // Servo for guide wheels
        guide = hardwareMap.servo.get("guide");

        // Initialize tasks, CapBallTask is for forklift initialization but is not actually used
        new CapBallTask(this);
        intakeTask = new IntakeTask(this);
        flywheelTask = new FlywheelTask(this);
        buttonPusherTask = new ButtonPusherTask(this);

        // Initialize the IMU, this will take a few seconds (majority of time)
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);

        // DriveTrain is used for majority of robot movement
        driveTrain = new MecanumDriveTrain(this);

        // Initialize with robot at the angle it would be when pressing beacons.
        // This is very important! The robot rotates back to this angle later on.
        wallAngle = imu.getAngle();

        // Bring guide position up
        guide.setPosition(ButtonPusherTask.upPosition);

        // Tell color sensor and drive train the team color, which is important for detecting the
        // team color (correctColor) and for driving orientations (powerUp, moveUp, etc)
        voiColorSensorTop.team = driveTrain.team = team;

        // Calculate current voltage for tasks. This determines the initial power that the motors
        // are set at.
        TaskThread.calculateVoltage(this);

        // Misc
        df.setMaximumFractionDigits(2);

    }

    public void shootFirstLineUp() {
        driveTrain.moveRightNInch(1, 10, 5, false, true);
        driveTrain.moveForwardNInch(0.5, 30, 5, false, true, false);
        driveTrain.moveRightNInch(1, 60, 10, true, true);
    }
    public void shootTwo() {
        driveTrain.moveBackwardNInch(0.2, 15, 5, false, true, false);
        flywheelTask.setFlywheelPow(shootFirstPower);
        sleep(2000);
        intakeTask.setPower(1);
        sleep(shootTime);
        intakeTask.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void pickUpBall(){
        /**
         * This method begins with the robot facing away from the corner vortex. The ball should be
         * right under the sweeper (but only touching the alliance robot). The goal is to pick up
         * the third ball, shoot all three balls, and end facing towards the white line of the
         * first beacon.
         * 
         * Steps:
         *  1. Run sweeper for 1.25 seconds.
         *  2. Strafe away from wall towards center vortex.
         *  3. Rotate so that flywheels are pointed towards center vortex.
         *  4. Run flywheels and sweeper to shoot balls.
         *  5. Rotate so that appropriate end is oriented towards white line of the first beacon.
         */
        
        // 1.
        int sweepTime = 1250;
        powerSweeper(1, sweepTime);
        sleep(sweepTime);
        
        // increase shootTime to account for third ball
        shootTime += 1000;

        // 2.
        if (shootFirst) {
            driveTrain.teamStrafeRightNInch(0.5, 1, 0.2, false, false);
            driveTrain.teamStrafeRightNInch(0.5, 20, 10, false, true);

            // 3.
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(wallAngle + 180);
            } else if (team == Team.RED) {
                driveTrain.rotateToAngle(wallAngle);
            }

            // 4.
            timer.reset();
            flywheelTask.setFlywheelPow(shootFirstPower);
            while (flywheelTask.getFlywheelState() != FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET && opModeIsActive()) {
                // TODO: 1/23/17 Figure out how to get the flywheel state from flywheeltask
                telemetry.addData("Time", (int) gameTimer.time());
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft * 100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight * 100));
                telemetry.addData("Flywheel state", flywheelTask.getFlywheelState());
                telemetry.update();
                if (timer.time() > 2000) {
                    // 5000 just for testing, change to 2000
                    break;
                }
            }
            intakeTask.setPower(1);
            sleep(shootTime);
            coolDown();
        } else {
            driveTrain.moveRightNInch(0.3, 5, 5, false, true);
            driveTrain.rotateToAngle(wallAngle);
        }

    }

    public void lineUpToWall(double distance) {
        /**
         * This method begins with the robot roughly pointing at the white line of the beacon.
         * The goal of this method is to end with the robot aligned with the wall, allowing it to
         * move along it while detecting and pushing beacons.
         *
         * Steps:
         *   1. Drive a set distance that should end end up with robot approximately at white line.
         *   2. Rotate so that robot is parallel with wall (to angle of initialization).
         *   3. Strafe towards the wall with considerable power to ensure lining up correctly.
         */
        telemetry.addData("lineUpToWall", "");
        telemetry.update();

        guide.setPosition(ButtonPusherTask.downPosition);
        int wlTimeout = 0; // timeout for white line detection (used for missing line)
        double fastDistance = distance - 0.5;
        double fastPower = 0.8;
        if (autoMode == AutoMode.Defensive) {
            fastPower = 1;
        }
        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false, false);
        buttonPusherTask.out();
        driveTrain.moveUpNInch(fastPower, fastDistance, 10, false, true, false);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveRightNInch(1, 60, 10, true, true);
    }

    public void drivePushButton() {
        
        /* This method assumes that the robot is already lined up to the wall and is
         * behind the first beacon. We do not want to spend time checking if the robot is in
         * front of the beacon, as that is unnecessary (and would be a fix in the lineUpToWall
         * method call.) The goal of the drivePushButton() method is to push the first beacon.
         * 
         * Steps:
         * 1. Activate the button pusher if the correct color is detected immediately.
         * 2. If the color sensor reads no color (or the wrong color), drive forward slowly until
         *    the correct color is detected and activate the button pusher
         * 3. If the top color sensor does not detect the correct color, then it assumes
         *    that the beacon turned the wrong color when ramming into the wall. Because of the 5s
         *    delay, the robot moves on to the second beacon and returns to the first beacon later.
         *    The "betweenBeacon" distance (how much the robot moves before starting to detect color
         *    again) is decreased to account for the extra distance moved in the timeout.
         */

        telemetry.addData("drivePushButton", "");
        telemetry.update();

        int timeOut = 3000;

        if (voiColorSensorTop.correctColor()) {
            // 1.
            pushButton();
            // If the correct color is detected immediately, then we assume that the we pressed
            // the sid farther from the second beacon. We increase the "betweenBeacon" distance to
            // account for that.
            betweenBeacon += 3;
            return;
        } else {
            // 2.
            timer.reset();
            boolean timeAdded = false;
            driveTrain.powerUp(bpPower);

            while (timer.time() < timeOut & opModeIsActive()) {
                if (voiColorSensorTop.wrongColor() && !timeAdded) {
                    // If the opposite color is detected, add 1 second to the timeout in case the
                    // robot starts further back than expected.
                    timeOut += 1000;
                    timeAdded = true;
                }
                // Turning because of known rotation after stopping. Temporary fix.
                // FIXME: 1/23/17 Solve rotation problem
                if (voiColorSensorTop.correctColor()) {
                    if (autoMode != AutoMode.Defensive) {
                        correctionStrafe();
                    }
                    pushButton();
                    return;
                }
            }
        }
        // 3.
        missed = true;
        betweenBeacon -= 8;
    }

    public void drivePushButton2() {
        /*  This method starts with the robot after it has pressed the first beacon (or given up.)
         *  The goal is to press the correct color on the second beacon.
         *
         *  Steps:
         *      1. Drive "betweenBeacon" distance before beginning to detect for the second beacon.
         *         This driving is done at a faster velocity to increase speed.
         *      2. Drive slower and look for beacon.
         *      3. When the correct color is detected, activate button pusher
         *
         */
        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 8000;
        double buffer = 8;
        // 1.
        driveTrain.moveUpNInch(0.4, betweenBeacon - buffer, 10, false, false, true);
        driveTrain.moveUpNInch(0.11, buffer, 3, false, false, true);

        // 2.
        driveTrain.powerUp(bpPower);
        boolean detectColor = false;
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        boolean far = false;
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // Boolean "far" determines the angle the robot turns to hit the cap ball.
                if (voiColorSensorTop.wrongColor() && !far) {
                    far = true;
                    timeo += 1000;
                }
                detectColor = voiColorSensorTop.correctColor();
                timer.reset();
            }
        }


        // 3.
        pushButton();
        if (far) {
            sFarRotB = sFarRotB2;
        }
        if (!missed) {
            // if all goes well, withdraw button pusher and lift guide wheels
            buttonPusherTask.in();
            guide.setPosition(ButtonPusherTask.upPosition);
        }
    }

    public void moveFromWall() {
        /**
         * This method starts right after the robot pushes the first beacon (after coming back from
         * the second beacon). The goal is to knock the cap ball off the center vortex and park.
         *
         * Steps:
         * 1. Strafe left from the wall, as you cannot rotate when right next to the wall.
         * 2. Rotate so that back of robot is oriented towards center vortex.
         * 3. Hit cap ball
         */
        telemetry.addData("moveFromWall", "");
        telemetry.update();

        // 1.
        driveTrain.moveLeftNInch(0.6, 6, 5, false, false);
        driveTrain.stopAll();

        // 2.
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sCloRotB);
        } else if (team == Team.RED){
            driveTrain.rotateToAngle(wallAngle + sCloRotR);
        }

        // 3.
        hitCapBall();
    }

    public void moveFromWall2() {
        /**
         * This method starts right after the robot has pressed the second beacon. The goal is to
         * point towards the center vortex and knock the cap ball off and park on the center vortex.
         *
         * Steps:
         * 1. Strafe away from the wall
         * 2. Rotate so that back is faced towards center vortex.
         * 3. Move backwards and knock the cap ball off.
         */

        telemetry.addData("moveFromWall2", "");
        telemetry.update();

        // lift up guide wheels
        guide.setPosition(ButtonPusherTask.upPosition);

        // 1.
        driveTrain.moveLeftNInch(0.6, 3, 10, false, true);

        // 2.
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        } else if (team == Team.RED) {
            sFarRotR += 175;
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }

        // 3.
        driveTrain.moveBackNInch(0.5, 30, 10, false, false, false);
        guide.setPosition(ButtonPusherTask.upPosition);
        driveTrain.moveBackNInch(0.5, 30, 10, false, true, false);

    }

    public void parkCorner() {
        guide.setPosition(ButtonPusherTask.upPosition);
        driveTrain.moveLeftNInch(0.5, 5, 5, false, true);
        sleep(200);
        if (autoMode == AutoMode.Defensive) {
            if (team == Team.BLUE) {
                driveTrain.rotateDegrees(VOIImu.subtractAngles(wallAngle, imu.getAngle() - 10), 0.3, true);
            } else if (team == Team.RED) {
                driveTrain.rotateDegrees(VOIImu.subtractAngles(wallAngle, imu.getAngle() + 10), 0.3, true);
            }
        }
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveBackNInch(1, 90, 10, false, true, false);
    }

    public void checkFirst() {
        /**
         * This method starts right after pressing the second beacon (and skipping the first).
         * The goal is to return to the first beacon and convert it to the alliance color.
         *
         * Steps:
         *  1. Move back a certain distance (to ensure that robot is behind the second beacon)
         *  2. Move back slowly while looking for the correct color.
         *  3. Activate button pusher when correct color detected.
         */
        telemetry.addData("checkFirst", "");

        shootPower = spalt;
        telemetry.update();
        // 1.
        driveTrain.moveBackNInch(0.3, betweenBeacon - 5, 10, false, false, true);
        driveTrain.moveBackNInch(0.15, 5, 10, false, true, true);
        correctionStrafe();

        // 2.
        driveTrain.powerUp(-bpPower);
        boolean isWrong = false;
        boolean rammedWrong = false;
        while (opModeIsActive() && !voiColorSensorTop.correctColor() && !rammedWrong) {
            if (voiColorSensorTop.wrongColor() && !isWrong) {
                isWrong = true;
                timer.reset();
            }
            if (isWrong && timer.time() > 1500) {
                // If, after detecting the wrong color for 1 second, the correct color is still not
                // detected, then we can infer that we activated the button pusher incorrectly upon
                // ramming.
                rammedWrong = true;
                driveTrain.stopAll();
            }
        }
        if (rammedWrong) {
            // go back and look for the wrong color to press
            correctionStrafe();
            driveTrain.powerUp(bpPower);
            while (opModeIsActive() && !voiColorSensorTop.wrongColor());
            pushButton();
        } else {
            // press the button if correct color is detected
            pushButton();
        }
        if (!isWrong) {
            shootRotation = sralt3;
        }

    }

    public void coolDown() {
        intakeTask.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        // Strafing against wall to ensure alignment. Same direction regardless of color because
        // button pusher is always on the right side.
        driveTrain.stopAll();
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveBackNInch(0.05, 1, 0.3, false, true, false);
        driveTrain.moveRightNInch(0.2, 5, seconds, false, true);
    }

    public void hitCapBall() {
        // FIXME: 1/24/17 rotation
        telemetry.addData("hitCapBall", "");
        telemetry.update();
        driveTrain.moveBackwardNInch(1, 50, 10, true, true, false);
        driveTrain.rotateDegrees(-270, 1, false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true, false);
    }

    public void pushButton() {
        // Stop driving and push the button. Wait a little more than the required push time to allow
        // button to partially retract.

        correctionStrafe();
        buttonPusherTask.push();
        sleep(buttonPusherTask.pushTime + 200);
    }

    public void powerSweeper(double power, int time) {
        intakeTask.power = power;
        intakeTask.sweepTime = time;
    }

    public void defense() {
        driveTrain.moveLeftNInch(1, 43, 10, false, true);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveUpNInch(0.5,20, 5, false, true, false);
        driveTrain.holdPosition();
        telemetry.addData("Time", gameTimer.time()*1.0/1000);
        System.out.println(gameTimer.time() * 1.0/1000);
        telemetry.update();
        while (gameTimer.time() < 30000 && opModeIsActive());

    }

    public void backToFirst() {
        driveTrain.moveBackNInch(0.4, betweenBeacon - 8, 10, false, false, true);
        driveTrain.moveBackNInch(0.11, 8, 5, false, true, true);
        driveTrain.powerUp(-bpPower);
        while (opModeIsActive() && !voiColorSensorTop.correctColor());
        driveTrain.stopAll();
    }

    public void options() {
        boolean confirmed = false;
        while(!confirmed){

            // select team
            if (gamepad1.b){
                team = Team.RED;
            } else if (gamepad1.x){
                team = Team.BLUE;
            }

            // select auto mode
            if (gamepad1.dpad_left) {
                autoMode = AutoMode.TwoBall;
            } else if (gamepad1.dpad_up) {
                autoMode = AutoMode.ThreeBall;
            } else if (gamepad1.dpad_right) {
                autoMode = AutoMode.Defensive;
            }

            // select park mode
            if (gamepad1.right_bumper) {
                parkMode = ParkMode.Corner;
            } else if (gamepad1.left_bumper) {
                parkMode = ParkMode.Center;
            }

            // select shoot order
            if (gamepad1.right_trigger > 0.15) {
                shootFirst = true;
            } else if (gamepad1.left_trigger > 0.15) {
                shootFirst = false;
            }

            telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
            telemetry.addData("Mode", autoMode);
            telemetry.addData("Park", parkMode);
            telemetry.addData("Shoot First", shootFirst ? "Yes" : "No");

            if ((gamepad1.left_stick_button && gamepad1.right_stick_button) || isStarted()){
                telemetry.addData("Confirmed!", "");
                voiColorSensorTop.team = driveTrain.team = team;
                confirmed = true;
                if (autoMode == AutoMode.Defensive) {
                    betweenBeacon += 3;
                    bpPower += 0.05;
                }
            }
            telemetry.update();

        }
    }

}
