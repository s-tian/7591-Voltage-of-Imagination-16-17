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
    public static final double TICKS_PER_INCH_FORWARD= 43.46;
    public static final double TICKS_PER_INCH_STRAFE = 48.97;
    public static final double TICKS_PER_MS_FORWARD = 2.3875; // power 1
    public static final double TICKS_PER_MS_STRAFE = 1.912; //  power 1
    public static final double factorFL = 1;
    public static final double factorFR = 1;//0.9072
    public static final double factorBL = 0.9069;//0.9069
    public static final double factorBR = 0.9323;

    //public ElapsedTime timer, timer2, timer3;
    public ElapsedTime timer;
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight, mWheel;
    //ModernRoboticsI2cGyro gyro;
    VOIImu imu;
    LinearOpMode opMode;

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

        this.imu = imu;
        this.opMode = opMode;
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mWheel = backRight;
        reverseMotors();
    }

    private void reverseMotors() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setEncoderMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }

    public void setMotorPower(DcMotor motor,double power){
        double factor = 1;
        if (motor == frontLeft){
            factor = factorFL;
        }else if (motor == frontRight){
            factor = factorFR;
        }else if (motor == backLeft){
            factor = factorBL;
        }else if (motor == backRight){
            factor = factorBR;
        }
        motor.setPower(POWER_RATIO*power*factor);
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
        setMotorPower(backRight,  -power);
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, -power);
        setMotorPower(frontRight, power);
    }

    public void strafeRight(double power) {
        setMotorPower(backRight, power);
        setMotorPower(backLeft, -power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, -power);
    }

    public void rotateDegreesPrecision(int degrees) {
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        double velocity, targetGyro = VOIImu.addAngles(imu.getAngle(), degrees);
        rotateDegrees((int)(degrees*0.8), true);
        while (Math.abs(VOIImu.subtractAngles(imu.getAngle(), targetGyro))> 1 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Angles",imu.getAngle()+ " " + targetGyro);
            opMode.updateTelemetry(opMode.telemetry);
            double gyroValue = imu.getAngle();
            if (VOIImu.subtractAngles(targetGyro, gyroValue) > 0)
                velocity = Math.max(VOIImu.subtractAngles(targetGyro, gyroValue)*0.2/degrees, 0.07);
            else
                velocity = Math.min(VOIImu.subtractAngles(targetGyro, gyroValue)*0.2/degrees, -0.07);
            startRotation(velocity);
        }
        stopAll();
    }

    public void rotateDegrees(int degrees, boolean slowdown) {
        double gyroValue = imu.getAngle();
        int targetGyro = VOIImu.addAngles(imu.getAngle(), degrees);
        double velocity;
        timer.reset();
        if (degrees < 0){

            startRotation(-0.30);
            while ((VOIImu.subtractAngles(targetGyro, gyroValue)) <  -2 && opMode.opModeIsActive()) {

                gyroValue = imu.getAngle();
                opMode.telemetry.addData("imu angle: " ,gyroValue);
                opMode.telemetry.addData("target angle: " ,targetGyro);
                if (slowdown) {
                    opMode.telemetry.addData("Angle left: ",VOIImu.subtractAngles(targetGyro, gyroValue, false));
                    velocity = Math.min(VOIImu.subtractAngles(targetGyro, gyroValue, false) * 0.15 / degrees, -0.2);
                    startRotation(velocity);
                }
                opMode.telemetry.update();

            }
        }
        else{
            startRotation(0.30);
            while ((VOIImu.subtractAngles(targetGyro, imu.getAngle())) > 2 && opMode.opModeIsActive()){
                gyroValue = imu.getAngle();
                opMode.telemetry.addData("imu angle: " ,gyroValue);
                opMode.telemetry.addData("target angle: " ,targetGyro);
                if (slowdown){
                    opMode.telemetry.addData("Angle left: ",VOIImu.subtractAngles(targetGyro, gyroValue, false));

                    //velocity = Math.max(subtractAngles(targetGyro, imu.getAngle(), true)*0.35/degrees, 0.2);
                    //startRotation(velocity);
                }
                opMode.telemetry.update();

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
        return moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall, stop);
    }

    public boolean moveRightNInch(double power, double inches, double timeout, boolean detectStall, boolean stop) {
        return moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall, stop);
    }

    public boolean moveLeftTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout*1000;
        int targetPosition = mWheel.getCurrentPosition() - ticks;
        strafeLeft(power);
        timer.reset();
        boolean startDetectingStall = false;
        long currentTime = System.currentTimeMillis();
        while(mWheel.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                }
                if (startDetectingStall && System.currentTimeMillis() - currentTime > 100) {
                    if (stalling(false)) {
                        return false;
                    }
                }
            }
        }
        if (timer.time() > timeOutMS) {
            stopAll();
        }
        if (timer.time() >= timeOutMS) {
            return false;
        }
        return true;
    }

    public boolean moveRightTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
        double timeOutMS = timeout * 1000;
        int targetPosition = mWheel.getCurrentPosition() + ticks;
        strafeRight(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while (mWheel.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 1000) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(false)) {
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

    public boolean moveForwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
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
                    if (stalling(true)) {
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

    public boolean moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall, boolean stop) {
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
                    if (stalling(true)) {
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

    public boolean moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        return moveBackwardTicksWithEncoders(power, ticks, timeout, detectStall, true);
    }

    public void getTicks(){
        System.out.println(mWheel.getCurrentPosition());
    }

    public boolean stalling(boolean forward) {
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
        int expected = (int) (TICKS_PER_MS_FORWARD * 100 / 2);

        if (!forward) {
            expected = (int) (TICKS_PER_MS_STRAFE * 100 * 0.5);
        }
        if (ticksBackLeft < expected) BLStall = true;
        if (ticksBackRight < expected) BRStall = true;
        if (ticksFrontRight < expected) FRStall = true;
        if (ticksFrontLeft < expected) FLStall = true;
        if (forward) {
            return (BLStall || FLStall) && (BRStall || FRStall);
        }
        return (BLStall || BRStall) && (FLStall || FRStall);

    }


}
