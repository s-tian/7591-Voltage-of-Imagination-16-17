package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static final double factorBRL = 0.44; // 0.42
    public static final double factorBLL = 1; //0.93;
    public static final double factorFRL = 1;//0.81;
    public static final double factorFLL = 0.44;
    public static final double ACF = 0.0025; // Angle Correction Factor
    public static final double changeFactor = 0.07;
    public Team team = Team.RED;
    //public ElapsedTime timer, timer2, timer3;
    public ElapsedTime timer;
    public ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight, mWheel;
    //ModernRoboticsI2cGyro gyro;
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

    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, LinearOpMode opMode) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.opMode = opMode;
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mWheel = frontLeft;

        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reverseMotors();
    }

    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, VOIImu imu, LinearOpMode opMode) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;
        this.opMode = opMode;
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mWheel = backRight;
        reverseMotors();
    }

    private void reverseMotors() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setEncoderMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }

    public void setMotorPower(DcMotor motor,double power){
        if (power >= 1) {
            motor.setPower(1);
        } else if (power <= -1) {
            motor.setPower(-1);
        } else {
            motor.setPower(power);
        }
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

    public void rotateToAngle (double target, double power) {
        double current = imu.getAngle();
        double difference = VOIImu.subtractAngles(target, current);
        rotateDegreesPrecision(difference, power);
    }

    public void rotateDegreesPrecision(double degrees) {
        rotateDegreesPrecision(degrees, 0.25);
    }

    public void rotateDegreesPrecision(double degrees, double power) {
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        boolean adjust = false;
        double velocity, targetGyro = VOIImu.addAngles(imu.getAngle(), degrees);
        rotateDegrees(degrees*0.8*(1 - power * 0.05), power, true);
        while (Math.abs(VOIImu.subtractAngles(imu.getAngle(), targetGyro))> 1 && opMode.opModeIsActive()){
            double gyroValue = imu.getAngle();
            if (VOIImu.subtractAngles(targetGyro, gyroValue) > 0) {
                if (adjust && degrees > 0) {
                    break;
                }
                if (degrees < 0) {
                    adjust = true;
                }
                velocity = Math.max(VOIImu.subtractAngles(targetGyro, gyroValue) * 0.2 / degrees, 0.08);
            } else {
                if (adjust = true && degrees < 0) {
                    break;
                }
                if (degrees > 0) {
                    adjust = true;
                }
                velocity = Math.min(VOIImu.subtractAngles(targetGyro, gyroValue) * 0.2 / degrees, -0.08);
            }
            startRotation(velocity);
        }
        stopAll();
    }

    public void rotateDegrees(double degrees, double power, boolean slowdown) {
        double gyroValue = imu.getAngle();
        double targetGyro = VOIImu.addAngles(imu.getAngle(), degrees);
        double velocity;
        timer.reset();
        if (degrees < 0){
            startRotation(-power);
            while ((VOIImu.subtractAngles(targetGyro, gyroValue)) <  -2 && opMode.opModeIsActive()) {
                gyroValue = imu.getAngle();
                if (slowdown) {
                    velocity = Math.min(VOIImu.subtractAngles(targetGyro, gyroValue) * power / degrees, -0.2);
                    startRotation(velocity);
                }
            }
        }
        else{
            startRotation(power);
            while ((VOIImu.subtractAngles(targetGyro, imu.getAngle())) > 2 && opMode.opModeIsActive()){
                gyroValue = imu.getAngle();
                if (slowdown){

                    velocity = Math.max(VOIImu.subtractAngles(targetGyro, gyroValue)*0.35/degrees, 0.2);
                    startRotation(velocity);
                }

            }
        }
        stopAll();
    }

    public boolean moveForwardNInch(double power, double inches, double timeout, boolean detectStall, boolean stop)  {
        return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
    }

    public boolean moveBackwardNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
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
        strafeLeft(power);
        timer.reset();
        timer2.reset();
        double maxDiff = 0;
        double initAngle = imu.getAngle();
        boolean startDetectingStall = false;
        long currentTime = System.currentTimeMillis();
        System.out.println("initial: " + initAngle);
        double current;
        double prev = initAngle;
        while(frontRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                }
                if (startDetectingStall && System.currentTimeMillis() - currentTime > 100) {
                    if (stalling(DIRECTION.LEFT)) {
                        stopAll();
                        return false;
                    }
                    currentTime = System.currentTimeMillis();

                }
            }
//            if (timer2.time() > 100) {
//                System.out.println("power: " + backRight.getPower());
//                System.out.println("current: " + imu.getAngle());
//                current = imu.getAngle();
//                double diff = VOIImu.subtractAngles(current, initAngle);
//                maxDiff = Math.max(Math.abs(diff), maxDiff);
//                double change = VOIImu.subtractAngles(current, prev);
//                double delta = diff * ACF + change * changeFactor;
//                double newBR = backRight.getPower() + delta;
//                double newFL = frontLeft.getPower() - delta;
//                double newBL = backLeft.getPower() - delta;
//                double newFR = frontRight.getPower() + delta;
//
//                backRight.setPower(newBR);
//                frontLeft.setPower(newFL);
//                backLeft.setPower(newBL);
//                frontRight.setPower(newFR);
//                prev = current;
//                timer2.reset();
//            }
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
        int targetPosition = mWheel.getCurrentPosition() + ticks;
        timer.reset();
        timer2.reset();
        double initAngle = imu.getAngle();
        strafeRight(power);
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while (mWheel.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 100) {
                    currentTime = System.currentTimeMillis();

                    if (stalling(DIRECTION.RIGHT)) {
                        stopAll();
                        return false;
                    }

                }
            }
//            if (timer2.time() > 100) {
//                double diff = imu.getAngle() - initAngle;
//                double newBR = backRight.getPower() + diff * ACF;
//                double newFL = frontLeft.getPower() + diff * ACF;
//                double newBL = backLeft.getPower() - diff * ACF;
//                double newFR = frontRight.getPower() - diff * ACF;
//
//                backRight.setPower(newBR);
//                frontLeft.setPower(newFL);
//                backLeft.setPower(newBL);
//                frontRight.setPower(newFR);
//                timer2.reset();
//            }

        }
        if (stop) {
            stopAll();
        }
        if (timer.time() >= timeOutMS) {
            return false;
        }
        return true;
    }

    private boolean moveForwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout*1000;
        int targetPosition = mWheel.getCurrentPosition() + ticks;
        powerAllMotors(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(mWheel.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
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

    private boolean moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout*1000;
        int targetPosition = mWheel.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(mWheel.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
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

    public void getTicks(){
        System.out.println(mWheel.getCurrentPosition());
    }

    public boolean moveUpNInch(double power, double inches, double timeout, boolean detectStall, boolean stop)  {
        if (team == Team.BLUE) {
            return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
        } else {
            return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
        }
    }

    public boolean moveBackNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        if (team == Team.BLUE) {
            return moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
        } else {
            return moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall, stop);
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
        while ((System.currentTimeMillis() - currentTime) < 100 && opMode.opModeIsActive()) {
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
                expected =  (TICKS_PER_MS_FORWARD * 100 * 0.5);
                break;
            case BACKWARD:
                expected = (TICKS_PER_MS_RIGHT *100 * 0.5);
                break;
            case LEFT:
                expected = TICKS_PER_MS_LEFT * 100 * 0.5;
                break;
            case RIGHT:
                expected = TICKS_PER_MS_FORWARD * 100 * 0.5;
                break;

        }
        if (ticksBackLeft < expected) BLStall = true;
        if (ticksBackRight < expected) BRStall = true;
        if (ticksFrontRight < expected) FRStall = true;
        if (ticksFrontLeft < expected) FLStall = true;
        switch (dir) {
            case FORWARD:
                return (BLStall || FLStall) && (BRStall && FRStall);
            case BACKWARD:
                return (BLStall || FLStall) && (BRStall && FRStall);
            case LEFT:
                return (FRStall || FLStall) && (BRStall && BLStall);
            case RIGHT:
                return (FRStall || FLStall) && (BRStall && BLStall);
        }
        return false;

    }

    public void printPower() {
        System.out.println("br " + backRight.getPower());
        System.out.println("bl " + backLeft.getPower());
        System.out.println("fr " + frontRight.getPower());
        System.out.println("fl " + frontLeft.getPower());
    }
}
