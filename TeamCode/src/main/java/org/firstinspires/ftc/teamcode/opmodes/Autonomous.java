package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

import java.text.DecimalFormat;

/**
 * Created by Howard on 12/13/16.
 * Autonomous
 */

@SuppressWarnings("ALL")
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

    // Options
    private boolean missed = false;
    private final int backID = 0x3a;
    private final int frontID = 0x3c;

    private int shootTime = 2500;

    private int betweenBeacon = 32; // far beacon distance

    // Angles
    private double shootRotation = 108; // first beacon shoot rotation near
    private double sralt3 = 90; // first beacons shoot rotation far
    private double wallAngle; // angle parallel to beacon wall
    int beaconRotation = 48;

    private double sFarRotB = 42; // shoot far rotation Blue near
    private double sFarRotB2 = 42; // shoot far rotation Blue far
    private double sFarRotR = 129; // shoot far rotation Red
    private double sCloRotB = 108; // shoot close rotation Blue
    private double sCloRotR = 80; // shoot close rotation Red

    // Powers
    private double shootPower = 0.7; // shoot first power
    private double bpPower = 0.1; // beacon pressing driveTrain power

    // Hardware
    private ColorSensor colorBack, colorFront;
    private VOIColorSensor voiColorBack, voiColorFront, voiFront, voiBack;
    // voiColor is the color sensor we use
    private Servo guide;
    private BNO055IMU adaImu;
    private VOIImu imu;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    //Misc
    private static Team team = Team.BLUE;
    private static AutoMode autoMode = AutoMode.ThreeBall;
    private static ParkMode parkMode = ParkMode.Center;
    private static int delayTime = 0;
    private static final boolean shootFirst = true;
    private FlywheelTask flywheelTask;
    private IntakeTask intakeTask;
    private ButtonPusherTask buttonPusherTask;

    private DecimalFormat df = new DecimalFormat();

    private MecanumDriveTrain driveTrain;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime colorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private enum AutoMode {
        TwoBall, ThreeBall, Defensive, JustShoot
    }

    private enum ParkMode {
        Corner, Center
    }

    @Override
    public void runOpMode() {
        initialize();


        options();
        waitForStart();
        System.out.println("Autonomous started!");
        gameTimer.reset();
        flywheelTask.start();
        buttonPusherTask.start();
        intakeTask.start();
        while (opModeIsActive() && gameTimer.time() < delayTime);
        if (autoMode == AutoMode.TwoBall || autoMode == AutoMode.ThreeBall) {
            runBalls();
        } else if (autoMode == AutoMode.JustShoot) {
            runJustShoot();
        } else if (autoMode == AutoMode.Defensive) {
            runJustShoot();
        }
    }
    
    public void initialize(){

        // Initialize color sensors
        colorBack = hardwareMap.colorSensor.get("colorBack");
        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorBack.setI2cAddress(I2cAddr.create8bit(backID));
        colorFront.setI2cAddress(I2cAddr.create8bit(frontID));
        voiColorBack = new VOIColorSensor(colorBack, this);
        voiColorFront = new VOIColorSensor(colorFront, this);
        voiColorBack.lightOn = false;
        voiColorFront.lightOn = false;
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
        driveTrain.bpPower = bpPower;
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        shootPower = FlywheelTask.lowPow;

        // Bring guide position up
        guide.setPosition(ButtonPusherTask.upPosition);

        // Tell color sensor and drive train the team color, which is important for detecting the
        // team color (correctColor) and for driving orientations (powerUp, moveUp, etc)
        voiColorBack.team = driveTrain.team = team;

        // Calculate current voltage for tasks. This determines the initial power that the motors
        // are set at.
        TaskThread.calculateVoltage(this);

        // Misc
        df.setMaximumFractionDigits(2);

    }

    public void runBalls() {
        if (autoMode == AutoMode.ThreeBall) {
            pickUp();
        } else if (autoMode == AutoMode.TwoBall) {
            flywheelTask.setFlywheelPow(shootPower);
            driveTrain.moveBackwardNInch(0.2, 15, 5, false, true, false);
        }
        if (shootFirst) {
            shoot();
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(VOIImu.addAngles(wallAngle, beaconRotation));
            } else if (team == Team.RED) {
                driveTrain.rotateToAngle(VOIImu.subtractAngles(wallAngle, beaconRotation));
            }
            lineUpToWall(40);
        } else {
            lineUpToWall(32);
        }
        drivePushButton();
        drivePushButton2();

        if (missed) {
            checkFirst();
            knockCap();
        } else {
            if (parkMode == ParkMode.Center) {
                knockCap2();
            } else if (parkMode == ParkMode.Corner) {
                parkCorner();
            }
        }
    }

    public void runDefensive() {
        // autonomous for driving between beacons of opposing side
        buttonPusherTask.out();
        lineUpToWall(40);
        drivePushButton();
        drivePushButton2();
        if (missed) {
            checkFirst();
            knockCap();
        } else {
            //defense();
        }
        buttonPusherTask.in();
        sleep(1000);

    }

    public void runJustShoot() {
        // start from far corner and shoot and knock cap ball
        driveTrain.moveBackwardNInch(0.3, 30, 10, false, true, false);
        flywheelTask.setFlywheelPow(shootPower);
        shoot();
        if (parkMode == ParkMode.Corner) {
            driveTrain.rotateToAngle(VOIImu.subtractAngles(wallAngle, 80));
            driveTrain.moveBackwardNInch(1, 75, 5, false, true, false);
        } else if (parkMode == ParkMode.Center) {
            driveTrain.moveBackwardNInch(0.3, 35, 5, false, true, false);
            if (autoMode == AutoMode.Defensive) {
                driveTrain.rotateToAngle(wallAngle);
                while (gameTimer.time() < 10000 && opModeIsActive()) ;
                driveTrain.moveUpNInch(1, 30, 10, false, true, false);
                sleep(200);
                driveTrain.rotateToAngle(wallAngle);
                driveTrain.moveLeftNInch(1, 10, 5, false, true);
                driveTrain.moveUpNInch(1, 40, 10, false, true, false);
                driveTrain.holdPosition();
                while (gameTimer.time() < 30000 & opModeIsActive()) ;
            }
        }
    }

    public void shoot() {
        flywheelTask.setPhoneRest();
        System.out.println("Shoot");
        timer.reset();
        timer2.reset();
        intakeTask.oscillate = true;
        int count = -100;
        while (opModeIsActive()) {
            if (flywheelTask.getFlywheelState() == FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET) {
                if (flywheelTask.count == count + 2) {
                    System.out.println(flywheelTask.count + " Good");
                    break;
                } else {
                    if (count == -100) {
                        count = flywheelTask.count;
                        System.out.println(count + " Good");
                    }
                }
            } else {
                count = -100;
            }
            if (timer.time() > 5000) {
                System.out.println("Flywheel time out");
                break;
            }
        }
        intakeTask.oscillate = false;
        intakeTask.power = 0;
        System.out.println("Start shooting");
        intakeTask.setPower(1);
        timer.reset();
        while (timer.time() < shootTime && opModeIsActive());
        coolDown();
    }

    public void pickUp() {
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
        flywheelTask.setFlywheelPow(shootPower + 0.015);
        int sweepTime = 1000;
        powerSweeper(1, sweepTime);
        sleep(sweepTime);
        //powerSweeper(-1, 650);
        //powerSweeper(-1, 250);
        // increase shootTime to account for third ball
        shootTime = 4000;

        // 2.
        if (shootFirst) {
            driveTrain.teamStrafeRightNInch(1, 15, 10, false, true, true);
            // 3.
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(wallAngle + 180, 0.25, 1.5, 6);
            } else if (team == Team.RED) {
                driveTrain.rotateToAngle(wallAngle, 0.25, 1.5, 6);
            }

            // 4.
        } else {
            driveTrain.moveRightNInch(1, 5, 5, false, true, true);
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

        buttonPusherTask.out();
        guide.setPosition(ButtonPusherTask.downPosition);
        int wlTimeout = 0; // timeout for white line detection (used for missing line)
        double fastDistance = distance - 0.5;
        double fastPower = 0.8;
        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false, false);
        driveTrain.moveUpNInch(fastPower, fastDistance, 10, false, true, false);
        sleep(50);
        driveTrain.rotateToAngle(wallAngle);
        sleep(50);
        if (team == Team.BLUE) {
            driveTrain.moveRightNInch(1, 60, 10, true, true, false);
        } else if (team == Team.RED) {
            driveTrain.moveRightNInch(1, 60, 10, true, true, true);
        }
        correctionStrafe();
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

        int timeOut = 2000;
        timer.reset();
        colorTimer.reset();
        if (voiBack.correctColor() &&  !voiFront.correctColor()) {
            System.out.println("First Condition: ");
            System.out.println("Back Color: " + voiBack.getColor());
            System.out.println("Front Color: " + voiFront.getColor());
            driveTrain.moveBackNInch(0.15, 1, 2, false, true, true);
            pushButton();
            return;
        } else if (voiFront.correctColor() && !voiBack.correctColor()) {
            System.out.println("Second Condition:");
            System.out.println("Back Color: " + voiBack.getColor());
            System.out.println("Front Color: " + voiFront.getColor());
            // 1.
            driveTrain.moveUpNInch(0.1, 1, 2, false, true, true);
            pushButton();
            // If the correct color is detected immediately, then we assume that the we pressed
            // the side farther from the second beacon. We increase the "betweenBeacon" distance to
            // account for that.
            betweenBeacon += 3;
            return;
        } else {
            // 2.
            driveTrain.crawlUp();
            timer.reset();
            boolean timeAdded = false;

            while (timer.time() < timeOut && opModeIsActive()) {
                if (voiBack.correctColor() && !voiFront.correctColor()) {
                    System.out.println("Back detect, front not, while drive");
                    driveTrain.moveBackNInch(0.15, 1, 2, false, true, true);
                    pushButton();
                    return;
                }
                if (voiFront.wrongColor() && !timeAdded) {
                    // If the opposite color is detected, add 1 second to the timeout in case the
                    // robot starts further back than expected.
                    timeOut += 1000;
                    timeAdded = true;
                }
                if (colorTimer.time() > 50) {
                    System.out.println("Driving: ");
                    System.out.println("Back Color: " + voiBack.getColor());
                    System.out.println("Front Color: " + voiFront.getColor());
                    colorTimer.reset();
                }
                // Turning because of known rotation after stopping. Temporary fix.
                if (voiFront.correctColor()) {
                    pushButton();

                    return;
                }

            }
        }
        
        // 3.
        missed = true;
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
        double buffer = 15;
        // 1.
        if (!missed) {
            driveTrain.moveUpNInch(0.4, betweenBeacon - buffer, 10, false, false, true);
        }

        driveTrain.moveUpNInch(0.10, buffer, 3, false, false, true);


        // 2.
        driveTrain.crawlUp();
        boolean detectColor = false;
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        boolean far = false;
        colorTimer.reset();
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // Boolean "far" determines the angle the robot turns to hit the cap ball.
                if (voiFront.wrongColor() && !far) {
                    far = true;
                    timeo += 1000;
                }

                detectColor = voiFront.correctColor();
                timer.reset();
            }
            if (colorTimer.time() > 50) {
                System.out.println("Driving 2: ");
                System.out.println("Back Color: " + voiBack.getColor());
                System.out.println("Front Color: " + voiFront.getColor());
                colorTimer.reset();
            }
        }


        // 3.
        pushButton();
        if (far) {
            sFarRotB = sFarRotB2;
        }
        if (!missed) {
            // if all goes well, withdraw button pusher and lift guide wheels
            guide.setPosition(ButtonPusherTask.upPosition);
        }
    }

    public void knockCap() {
        /**
         * This method starts right after the robot pushes the first beacon (after coming back from
         * the second beacon). The goal is to knock the cap ball off the center vortex and park.
         *
         * Steps:
         * 1. Strafe left from the wall, as you cannot rotate when right next to the wall.
         * 2. Rotate so that back of robot is oriented towards center vortex.
         * 3. Hit cap ball
         */
        buttonPusherTask.in();
        telemetry.addData("knockCap", "");
        telemetry.update();

        // 1.
        driveTrain.moveLeftNInch(1, 6, 5, false, false);
        driveTrain.stopAll();
        guide.setPosition(ButtonPusherTask.upPosition);
        // 2.
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sCloRotB);
        } else if (team == Team.RED){
            driveTrain.rotateToAngle(wallAngle + sCloRotR);
        }
        
        //3.
        driveTrain.moveBackwardNInch(1, 50, 10, true, true, false);
        driveTrain.rotateDegrees(-270, 1, false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true, false);
    }

    public void knockCap2() {
        /**
         * This method starts right after the robot has pressed the second beacon. The goal is to
         * point towards the center vortex and knock the cap ball off and park on the center vortex.
         *
         * Steps:
         * 1. Strafe away from the wall
         * 2. Rotate so that back is faced towards center vortex.
         * 3. Move backwards and knock the cap ball off.
         */

        telemetry.addData("knockCap2", "");
        telemetry.update();
        buttonPusherTask.in();
        // lift up guide wheels
        guide.setPosition(ButtonPusherTask.upPosition);

        // 1.
        driveTrain.moveLeftNInch(1, 3, 10, false, true);

        // 2.
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        } else if (team == Team.RED) {
            sFarRotR += 180;
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }

        // 3.
        driveTrain.moveBackNInch(0.5, 50, 10, false, false, false);
        driveTrain.stopAll();

    }
    
    public void parkCorner() {
        /**
         * This method starts right after the robot has pressed the second beacon. The goal is to
         * park onto the corner vortex (only run when the alliance robot can knock off cap ball)
         *
         * Steps:
         * 1. Strafe away from the wall
         * 2. Rotate so that back is faced towards center vortex.
         * 3. Move backwards and knock the cap ball off.
         */
        buttonPusherTask.in();
        driveTrain.moveLeftNInch(1, 12, 5, false, true);
        sleep(200);
        driveTrain.rotateToAngle(wallAngle);
        guide.setPosition(ButtonPusherTask.upPosition);
        double tiltRoll = imu.getRoll() + 10;
        driveTrain.moveBackNInch(1, 90, 10, false, true, false);
        if (team == Team.BLUE) {
            double target = backRight.getCurrentPosition() + 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() < target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        } else if (team == Team.RED) {
            double target = backRight.getCurrentPosition() - 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() > target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        }
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

        telemetry.update();
        // 1.
        driveTrain.moveBackNInch(0.3, betweenBeacon - 5, 10, false, false, true);
        driveTrain.moveBackNInch(0.15, 5, 10, false, true, true);
        correctionStrafe();

        // 2.
        driveTrain.crawlBack();
        boolean isWrong = false;
        boolean rammedWrong = false;
        while (opModeIsActive() && !voiBack.correctColor() && !rammedWrong) {
            if (voiBack.wrongColor() && !isWrong) {
                isWrong = true;
                timer.reset();
            }
            if (isWrong && timer.time() > 750) {
                // If, after detecting the wrong color for some time, the correct color is still not
                // detected, then we can infer that we activated the button pusher incorrectly upon
                // ramming.
                rammedWrong = true;
                driveTrain.stopAll();
            }
        }
        if (rammedWrong) {
            // go back and look for the wrong color to press
            correctionStrafe();
            driveTrain.crawlUp();
            while (opModeIsActive() && !voiBack.wrongColor());
            pushButton();
        } else {
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
        if (team == Team.RED) {
            driveTrain.rotateToAngle(wallAngle, 0.25, 2, 0.75);
        }
        driveTrain.moveRightNInch(0.2, 5, seconds, false, true, false);
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
        driveTrain.crawlBack();
        while (opModeIsActive() && !voiFront.correctColor());
        driveTrain.stopAll();
    }

    public void options() {
        boolean confirmed = false;
        boolean rTrigger = false;
        boolean lTrigger = false;
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
            } else if (gamepad1.dpad_down) {
                autoMode = AutoMode.JustShoot;
            }

            // select park mode
            if (gamepad1.right_bumper) {
                parkMode = ParkMode.Center;
            } else if (gamepad1.left_bumper) {
                parkMode = ParkMode.Corner;
            }

            if (gamepad1.right_trigger > 0.15 && !rTrigger) {
                rTrigger = true;
                delayTime += 500;
            }
            if (gamepad1.left_trigger > 0.15 && !lTrigger) {
                lTrigger = true;
                delayTime -= 500;
            }
            if (gamepad1.right_trigger < 0.15) {
                rTrigger = false;
            }
            if (gamepad1.left_trigger < 0.15) {
                lTrigger = false;
            }
            telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
            telemetry.addData("Mode", autoMode);
            telemetry.addData("Park", parkMode);
            telemetry.addData("Delay", (double)delayTime/1000);

            if ((gamepad1.left_stick_button && gamepad1.right_stick_button) || isStarted()){
                telemetry.addData("Confirmed!", "");
                if (team == Team.BLUE) {
                    voiFront = voiColorFront;
                    voiBack = voiColorBack;
                } else if (team == Team.RED){
                    voiFront = voiColorBack;
                    voiBack = voiColorFront;
                }
                voiFront.team = driveTrain.team = voiBack.team = team;
                confirmed = true;
                if (autoMode == AutoMode.JustShoot) {
                    betweenBeacon += 3;
                    bpPower += 0.05;
                    wallAngle = imu.getAngle();
                } else if (autoMode == AutoMode.TwoBall) {

                    if (team == Team.RED) {
                        wallAngle = imu.getAngle();
                    } else if (team == Team.BLUE) {
                        wallAngle = VOIImu.addAngles(wallAngle, 180);
                    }
                } else if (autoMode == AutoMode.ThreeBall) {
                    // same for both sides
                    wallAngle = VOIImu.addAngles(imu.getAngle(), 90);
                } else if (autoMode == AutoMode.Defensive) {
                    wallAngle = imu.getAngle();
                    shootTime = 1500;
                }
            }
            telemetry.update();

        }
    }

    public void beaconTest() {
        for (int i = 0; i < 10; i ++) {
            if (!opModeIsActive()) {
                break;
            }
            buttonPusherTask.pushTime = 400;
            betweenBeacon = 33;
            guide.setPosition(ButtonPusherTask.downPosition);
            buttonPusherTask.out();
            sleep(1000);
            gameTimer.reset();
            drivePushButton();
            drivePushButton2();
            System.out.println(gameTimer.time());
            buttonPusherTask.in();
            sleep(1000);
            driveTrain.moveBackNInch(0.25, 55, 10, false, true, true);
            flywheelTask.running = false;
        }
    }


}