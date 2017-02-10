package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {
    //export PATH="$PATH:/Users/Howard/Library/Android/sdk/platform-tools"
    public static final double TICKS_PER_INCH_FORWARD= 42.437;
    public static final double TICKS_PER_INCH_RIGHT = 46.731;
    public static final double TICKS_PER_INCH_LEFT = 52.5;
    public static final double TICKS_PER_MS_FORWARD = 1.4; // power 1
    public static final double TICKS_PER_MS_RIGHT = 1.1649; //  power 1
    public static final double TICKS_PER_MS_LEFT = 0.979;
    public static final double factorFL = 1;
    public static final double factorFR = 1;
    public static final double factorBL = 1;
    public static final double factorBR = 1;
    public static final double factorBRL = 1; // 0.42
    public static final double factorBLL = 1; //0.93;
    public static final double factorFRL = 1;//0.81;
    public static final double factorFLL = 1;

    public static final int angleBuffer = 30;
    public static double stallTime = 20;
    public static double KP = 0.00; // Angle Correction Factor
    public static double KD = 0.00;
    public static double KI = 0;
    public double bpPower = 0.11;

    public static final double minRotPow = 0.05; // Minimum rotation power
    public Team team = Team.RED;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight;
    VOIImu imu;
    LinearOpMode opMode;

    public enum DIRECTION {
        FORWARD, RIGHT, LEFT, BACKWARD;

        @Override
        public String toString() {
            switch (this) {
                case FORWARD:
                    return "FORWARD";
                case RIGHT:
                    return "RIGHT";
                case LEFT:
                    return "LEFT";
                case BACKWARD:
                    return "BACKWARD";
            }
            return "";

        }
    }

    public MecanumDriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;
        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        BNO055IMU adaImu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reverseMotors();
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopAll();
    }

    private void reverseMotors() {
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setEncoderMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        switch (runMode) {
            case RUN_USING_ENCODER:
                powerAllMotors(0);
                break;
            case RUN_TO_POSITION:
                powerAllMotors(1);
                break;
            default:
                break;
        }
    }

    public void setMotorPower(DcMotor motor,double power){
        power = Range.clip(power, -1, 1);
        motor.setPower(power);
    }

    public void powerAllMotors(double power){
        setMotorPower(backLeft, power);
        setMotorPower(backRight, power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, power);
    }

    public void stopAll() {
        powerAllMotors(0);
    }

    public void powerRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(frontRight, power);
    }

    public void powerLeft(double power){
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, power);
    }

    public void startRotation(double power){
        // power > 0 : clockwise
        // power < 0 : counterclockwise
        powerLeft(power);
        powerRight(-power);
    }

    public void strafeLeft(double power) {
        setMotorPower(backRight,  -power*factorBRL);
        setMotorPower(backLeft, power*factorBLL);
        setMotorPower(frontLeft, -power*factorFLL);
        setMotorPower(frontRight, power*factorFRL);
    }

    public void strafeRight(double power) {
        setMotorPower(backRight, power*factorBR);
        setMotorPower(backLeft, -power*factorBL);
        setMotorPower(frontLeft, power*factorFL);
        setMotorPower(frontRight, -power*factorFR);
    }

    public void rotateToAngle(double target) {
        rotateToAngle(target, 0.25);
    }

    public void rotateToAngle(double target, double power) {
        double current = imu.getAngle();
        double difference = VOIImu.subtractAngles(target, current);
        rotateDegreesPrecision(difference, power);
    }

    public void rotateDegreesPrecision(double degrees, double power) {
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;

        // because consistently overshoots by 1 degree
        boolean adjust = false;
        double velocity;
        double target = VOIImu.addAngles(imu.getAngle(), degrees);
        if (degrees < -angleBuffer) {
            //degrees += 2;
            rotateDegrees(VOIImu.subtractAngles(degrees, -angleBuffer), power, false);
        } else if (degrees > angleBuffer) {
            //degrees -= 2;
            rotateDegrees(VOIImu.addAngles(degrees, - angleBuffer), power, false);
        }
        //while (VOIImu.subtractAngles(imu.getAngle(), target) > 1);
        while (Math.abs(VOIImu.subtractAngles(imu.getAngle(), target)) > 3 && opMode.opModeIsActive()){
            double gyroValue = imu.getAngle();
            if (VOIImu.subtractAngles(target, gyroValue) > 0) {
                if (adjust && degrees > 0) {
                    break;
                }
                if (degrees < 0) {
                    adjust = true;
                }
                velocity = Math.max(VOIImu.subtractAngles(target, gyroValue) * minRotPow * 2 / degrees, minRotPow);
            } else {
                if (adjust && degrees < 0) {
                    break;
                }
                if (degrees > 0) {
                    adjust = true;
                }
                velocity = Math.min(VOIImu.subtractAngles(target, gyroValue) * 0.2 / degrees, -minRotPow);
            }
            startRotation(velocity);
        }
        stopAll();
    }

    public void rotateDegrees(double degrees, double power, boolean stop) {
        double gyroValue = imu.getAngle();
        double target = VOIImu.addAngles(imu.getAngle(), degrees);
        timer.reset();
        double diff = VOIImu.subtractAngles(target, imu.getAngle());
        if (diff < 0){
            startRotation(-power);
            while ((VOIImu.subtractAngles(target, imu.getAngle())) <  -2 && opMode.opModeIsActive());
        } else{
            startRotation(power);
            while ((VOIImu.subtractAngles(target, imu.getAngle())) > 2 && opMode.opModeIsActive());
        }
        if (stop) {
            stopAll();
        }
    }

    public void crawlUp() {
        powerUp(bpPower);
        // boost speeds to make it stay on wall.
        if (team == Team.BLUE) {
            backLeft.setPower(bpPower * 1.3);
            frontLeft.setPower(bpPower * 1.3);
        } else if (team == Team.RED) {
            backRight.setPower(-bpPower * 1.0);
            frontLeft.setPower(-bpPower * 1.0);
        }
    }

    public void crawlBack() {
        powerUp(-bpPower);
        // boost speeds to make it stay on wall.
        if (team == Team.RED) {
            backLeft.setPower(bpPower * 1.3);
            frontLeft.setPower(bpPower * 1.3);
        } else if (team == Team.BLUE) {
            backRight.setPower(-bpPower * 1.0);
            frontLeft.setPower(-bpPower * 1.0);
        }
    }

    /**
     *
     * @param power the power at which the robot runs
     * @param inches distance robot should move
     * @param timeout time after which movement terminates
     * @param detectStall if robot should stop if detecting stall
     * @param stop if robot should stop after reaching distance
     * @return true if inches reached, false if terminated due to stall or timeout
     */
    public boolean moveForwardNInch(double power, double inches, double timeout, boolean detectStall, boolean stop, boolean lean)  {
        return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
    }

    public boolean moveBackwardNInch(double power, double inches, double timeout, boolean detectStall, boolean stop, boolean lean) {
        return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
    }

    public boolean moveLeftNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        return moveLeftTicksWithEncoders(power, (inches*TICKS_PER_INCH_LEFT), timeout, detectStall, stop);
    }

    public boolean moveRightNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        return moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_RIGHT), timeout, detectStall, stop);
    }

    private boolean moveLeftTicksWithEncoders(double power, double ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout*1000;
        double targetPosition = frontRight.getCurrentPosition() + ticks;
        timer.reset();
        timer2.reset();
        backRight.setPower(-1);
        backLeft.setPower(1);
        while (opMode.opModeIsActive() && timer.time() < stallTime);
        strafeLeft(power);

        timer.reset();
        timer2.reset();
        double maxDiff = 0;
        double initAngle = imu.getAngle();
        boolean startDetectingStall = false;
        long currentTime = System.currentTimeMillis();
        double current;
        double prev = initAngle;
        while(frontRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                }
                if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    if (stalling(DIRECTION.LEFT)) {
                        stopAll();
                        return false;
                    }
                    currentTime = System.currentTimeMillis();

                }
            }
            if (timer2.time() > 25) {
                current = imu.getAngle();
                double diff = VOIImu.subtractAngles(current, prev);
                maxDiff = Math.max(Math.abs(diff), maxDiff);
                double change = VOIImu.subtractAngles(current, prev);
                double delta = diff * KP + change * KD;
                double newBR = backRight.getPower() + delta;
                double newFL = frontLeft.getPower() - delta;
                double newBL = backLeft.getPower() - delta;
                double newFR = frontRight.getPower() + delta;

                backRight.setPower(newBR);
                frontLeft.setPower(newFL);
                backLeft.setPower(newBL);
                frontRight.setPower(newFR);
                prev = current;
                timer2.reset();
                opMode.telemetry.addData("change", change);
                opMode.telemetry.update();
            }
        }
        if (stop) {
            stopAll();
        }
        if (timer.time() >= timeOutMS) {
            return false;
        }
        return true;
    }

    private boolean moveRightTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout * 1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        timer.reset();
        timer2.reset();
        double initAngle = imu.getAngle();
        backRight.setPower(1);
        backLeft.setPower(-1);
        while (opMode.opModeIsActive() && timer.time() < stallTime);
        timer.reset();
        timer2.reset();
        strafeRight(power);
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while (backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();

                    if (stalling(DIRECTION.RIGHT)) {
                        stopAll();
                        return false;
                    }

                }
            }
            if (timer2.time() > 25) {
                double diff = imu.getAngle() - initAngle;
                double newBR = backRight.getPower() + diff * KP;
                double newFL = frontLeft.getPower() + diff * KP;
                double newBL = backLeft.getPower() - diff * KP;
                double newFR = frontRight.getPower() - diff * KP;

                backRight.setPower(newBR);
                frontLeft.setPower(newFL);
                backLeft.setPower(newBL);
                frontRight.setPower(newFR);
                timer2.reset();
            }

        }
        if (stop) {
            stopAll();
        }
        if (timer.time() >= timeOutMS) {
            return false;
        }
        return true;
    }

    private boolean moveForwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop, boolean lean) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        powerAllMotors(power);
        if (lean) {
            double boostPower = power * 1.3;
            backRight.setPower(boostPower);
            frontLeft.setPower(boostPower);
        }
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(DIRECTION.FORWARD)) {
                        stopAll();
                        return false;
                    }
                }
            }
        }
        if (stop) {
            stopAll();
        }
        if (timer.time() >= timeOutMS) {
            return false;
        }
        return true;
    }

    private boolean moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop, boolean lean) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        if (lean) {
            double boostPower = -power;
            backLeft.setPower(boostPower);
            frontRight.setPower(boostPower);
        }
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(DIRECTION.BACKWARD)) {
                        System.out.println("Stalling " + timeout);
                        stopAll();
                        return false;
                    }
                }
            }
        }
        if (stop) {
            stopAll();
        }
        if (timer.time() > timeOutMS) {
            return false;
        }
        return true;
    }

    public boolean moveUpNInch(double power, double inches, double timeout, boolean detectStall, boolean stop, boolean lean)  {
        if (team == Team.BLUE) {
            return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
        } else {
            return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
        }
    }

    public boolean moveBackNInch(double power, double inches, double timeout, boolean detectStall, boolean stop, boolean lean) {
        if (team == Team.BLUE) {
            return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
        } else {
            return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop, lean);
        }
    }

    public boolean teamStrafeLeftNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        if (team == Team.BLUE) {
            return moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_LEFT), timeout, detectStall, stop);
        } else {
            return moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_RIGHT), timeout, detectStall, stop);
        }
    }

    public boolean teamStrafeRightNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        if (team == Team.BLUE) {
            return moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_RIGHT), timeout, detectStall, stop);
        } else {
            return moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_LEFT), timeout, detectStall, stop);
        }
    }

    public void powerUp(double power) {
        if (team == Team.BLUE) {
            powerAllMotors(power);
        } else {
            powerAllMotors(-power);
        }
    }
    
    public boolean stalling(DIRECTION dir) {
        int initialBackRight = backRight.getCurrentPosition();
        int initialFrontRight = frontRight.getCurrentPosition();
        int initialBackLeft = backLeft.getCurrentPosition();
        int initialFrontLeft = frontLeft.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentTime) < 50 && opMode.opModeIsActive()) {
        }
        int ticksBackRight = Math.abs(backRight.getCurrentPosition() - initialBackRight);
        int ticksFrontRight = Math.abs(frontRight.getCurrentPosition() - initialFrontRight);
        int ticksBackLeft = Math.abs(backLeft.getCurrentPosition() - initialBackLeft);
        int ticksFrontLeft = Math.abs(frontLeft.getCurrentPosition() - initialFrontLeft);
        boolean BRStall = false;
        boolean BLStall = false;
        boolean FRStall = false;
        boolean FLStall = false;
        double expected = 0;

        switch (dir) {
            case FORWARD:
                expected =  TICKS_PER_MS_FORWARD * 50 * 0.3;
                break;
            case BACKWARD:
                expected = TICKS_PER_MS_RIGHT * 50 * 0.3;
                break;
            case LEFT:
                expected = TICKS_PER_MS_LEFT * 50 * 0.3;
                break;
            case RIGHT:
                expected = TICKS_PER_MS_FORWARD * 50 * 0.3;
                break;

        }
        if (ticksBackLeft < expected) BLStall = true;
        if (ticksBackRight < expected) BRStall = true;
        if (ticksFrontRight < expected) FRStall = true;
        if (ticksFrontLeft < expected) FLStall = true;
        switch (dir) {
            case FORWARD:
                return (BLStall || FLStall) && (BRStall || FRStall);
            case BACKWARD:
                return (BLStall || FLStall) && (BRStall || FRStall);
            case LEFT:
                return (FRStall || FLStall) && (BRStall || BLStall);
            case RIGHT:
                return (FRStall || FLStall) && (BRStall || BLStall);
        }
        return false;

    }

    public void holdPosition() {
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        powerAllMotors(1);

    }

    public void driveToPosition(int br, int bl, int fr, int fl) {
        //setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean brGood = false;
        boolean blGood = false;
        boolean frGood = false;
        boolean flGood = false;
        int error = 100;
        backRight.setTargetPosition(br);
        backLeft.setTargetPosition(bl);
        frontRight.setTargetPosition(fr);
        frontLeft.setTargetPosition(fl);
        powerAllMotors(0.1);
        int a = 0;
        while (a < 3) {
            if (!opMode.opModeIsActive()) {
                break;
            }//backRight: 153 backLeft: 35 frontRight: 731 frontLeft: 680
            a = 0;
            brGood = Math.abs(br - backRight.getCurrentPosition()) < error;
            blGood = Math.abs(bl - backLeft.getCurrentPosition()) < error;
            frGood = Math.abs(fr - frontRight.getCurrentPosition()) < error;
            flGood = Math.abs(fl - frontLeft.getCurrentPosition()) < error;

            if (brGood)
                a ++;
            if (blGood)
                a++;
            if (frGood)
                a++;
            if (flGood)
                a++;
            if (timer.time() > 10) {
                System.out.print("backRight: " + backRight.getCurrentPosition());
                System.out.print(" backLeft: " + backLeft.getCurrentPosition());
                System.out.print(" frontRight: " + frontRight.getCurrentPosition());
                System.out.println(" frontLeft: " + frontLeft.getCurrentPosition());
                timer.reset();
            }

        }

        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopAll();
    }
}
