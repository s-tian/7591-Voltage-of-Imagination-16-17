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
    public static final double TICKS_PER_INCH_FORWARD= 57.89;
    public static final double TICKS_PER_INCH_STRAFE = 61.05;
    public static final double TICKS_PER_MS_FORWARD = 2.489; // at power = 1
    public static final double TICKS_PER_MS_STRAFE = 1.961; // at power = 1
    public static final double factorFL = 1;
    public static final double factorFR = 1;//0.9072
    public static final double factorBL = 0.9069;//0.9069
    public static final double factorBR = 0.9323;

    //public ElapsedTime timer, timer2, timer3;
    public ElapsedTime timer;
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight;
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

        reverseMotors();
    }
    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //this.gyro = gyro;
        this.opMode = opMode;
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
        reverseMotors();
    }
    private void reverseMotors() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double velocity, targetGyro = addAngles(imu.getAngle(), degrees);
        rotateDegrees(degrees, true);
        while (Math.abs(subtractAngles(imu.getAngle(), targetGyro))> 2 && opMode.opModeIsActive()){
            double gyroValue = imu.getAngle();
            if (subtractAngles(targetGyro, gyroValue) > 0)
                velocity = Math.max(subtractAngles(targetGyro, gyroValue)*0.7/degrees, 0.1);
            else
                velocity = Math.min(subtractAngles(targetGyro, gyroValue)*0.7/degrees, -0.1);
            startRotation(velocity);
        }
        stopAll();
        System.out.println(imu.getAngle());
    }
    public void rotateDegrees(int degrees, boolean slowdown) {
        double gyroValue = imu.getAngle();
        int targetGyro = addAngles(imu.getAngle(), degrees);
        double velocity;
        timer.reset();
        if (degrees < 0){

            startRotation(-0.35);
            while ((subtractAngles(targetGyro, gyroValue)) <  -2 && opMode.opModeIsActive()) {

                gyroValue = imu.getAngle();
                opMode.telemetry.addData("imu angle: " ,gyroValue);
                opMode.telemetry.addData("target angle: " ,targetGyro);
                if (slowdown) {
                    opMode.telemetry.addData("Angle left: ",subtractAngles(targetGyro, gyroValue, false));
                    //velocity = Math.min(subtractAngles(targetGyro, gyroValue, false) * 0.35 / degrees, -0.2);
                    //startRotation(velocity);
                }
                opMode.telemetry.update();

            }
        }
        else{
            startRotation(0.35);
            while ((subtractAngles(targetGyro, imu.getAngle())) > 2 && opMode.opModeIsActive()){
                gyroValue = imu.getAngle();
                opMode.telemetry.addData("imu angle: " ,gyroValue);
                opMode.telemetry.addData("target angle: " ,targetGyro);
                if (slowdown){
                    opMode.telemetry.addData("Angle left: ",subtractAngles(targetGyro, gyroValue, false));

                    //velocity = Math.max(subtractAngles(targetGyro, imu.getAngle(), true)*0.35/degrees, 0.2);
                    //startRotation(velocity);
                }
                opMode.telemetry.update();

            }
        }
        stopAll();
    }
    public void moveForwardNInch(double power, double inches, double timeout, boolean detectStall)  {
        moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall);
    }
    public void moveBackwardNInch(double power, double inches, double timeout, boolean detectStall) {
        System.out.println(inches);
        moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD), timeout, detectStall);
    }
    public void moveLeftNInch(double power, double inches, double timeout, boolean detectStall) {
        moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall);
}
    public void moveRightNInch(double power, double inches, double timeout, boolean detectStall) {
        moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout, detectStall);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        strafeLeft(power);
        timer.reset();
        boolean startDetectingStall = false;
        long currentTime = System.currentTimeMillis();
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 500) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                }
                if (startDetectingStall && System.currentTimeMillis() - currentTime > 100) {
                    if (stalling(false)) {
                        return;
                    }
                }
            }
        }
        timer.reset();
        stopAll();
    }
    public void moveRightTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        strafeRight(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 300) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(false)) {
                        stopAll();
                        return;
                    }
                }
            }
        }
        stopAll();
    }
    public void moveForwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        powerAllMotors(power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 300) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(true)) {
                        stopAll();
                        return;
                    }
                }
            }
        }
        stopAll();
    }
    public void moveBackwardTicksWithEncoders(double power, int ticks, double timeout, boolean detectStall) {
        System.out.println("Start " + timeout );
        System.out.println(backRight.getCurrentPosition());
        double timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        timer.reset();
        long currentTime = System.currentTimeMillis();
        boolean startDetectingStall = false;
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {
            if (detectStall) {
                if (System.currentTimeMillis() - currentTime > 500) {
                    startDetectingStall = true;
                    currentTime = System.currentTimeMillis();
                } else if (startDetectingStall && System.currentTimeMillis() - currentTime > 50) {
                    currentTime = System.currentTimeMillis();
                    if (stalling(true)) {
                        System.out.println("Stalling " + timeout);
                        stopAll();
                        return;
                    }
                }
            }
        }

        System.out.println("Normal stop " + timeout);
        stopAll();
        System.out.println(backRight.getCurrentPosition());
        System.out.println("End");
    }
    public void getTicks(){
        System.out.println(backRight.getCurrentPosition());
    }
    public void powerForTime(double power, double time) {
        double timeMS = time*1000;
        long currentTime = System.currentTimeMillis();
        powerAllMotors(power);
        while (opMode.opModeIsActive() && (System.currentTimeMillis()-currentTime) < timeMS){
        }
        stopAll();
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
        int stallingMotors = 0;
        if (ticksBackLeft < expected) BLStall = true;
        if (ticksBackRight < expected) BRStall = true;
        if (ticksFrontRight < expected) FRStall = true;
        if (ticksFrontLeft < expected) FLStall = true;
        if (forward) {
            return (BLStall || FLStall) && (BRStall || FRStall);
        }
        return (BLStall || BRStall) && (FLStall || FRStall);

    }
    public int addAngles(int angle1, int angle2){
        int sum = (angle1 + angle2)%360;
        if (sum >= 180){
            sum -= 360;
        } else if (sum <= -180){
            sum += 360;
        }
        return sum;
    }
    public int subtractAngles(double angle1, double angle2){
        if (angle1 < 0) {
            angle1 += 360;
        }
        if (angle2 < 0) {
            angle2 += 360;
        }
        int diff1 = (int)(angle1 - angle2);
        int diff2 = diff1 + 360;
        int diff3 = diff1 - 360;
        if (Math.abs(diff1) <= Math.abs(diff2) && Math.abs(diff1) <= Math.abs(diff3)){
            return diff1;
        } else if (Math.abs(diff2) < Math.abs(diff3)){
            return diff2;
        }
        return diff3;
    }

    public int subtractAngles(double angle1, double angle2, boolean clockwise) {
        // angle1 - angle2
        if (angle1 < 0) {
            angle1 += 360;
        }
        if (angle2 < 0) {
            angle2 += 360;
        }
        if (clockwise && angle1 < angle2) {
            angle1 += 360;
        } else if (!clockwise && angle1 > angle2) {
            angle1 -= 360;
        }
        return (int)(angle1 - angle2);

    }
}
