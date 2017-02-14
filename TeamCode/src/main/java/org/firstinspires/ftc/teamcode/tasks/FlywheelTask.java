package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * Created by Howard on 10/15/16.
 */
public class FlywheelTask extends TaskThread {

    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    public volatile FlywheelState state;
    private final int THEORETICAL_MAX_RPM = 1800;
    private final int FULL_SPEED_RPM = 1300;
    private final double MAXPOWER = 0.42;
    private final int TICKS_PER_REV = 112;
    private final double MAX_ENCODER_TICKS_PER_MS = 2.9;
    private final double MAX_ALLOWED_ERROR = 0.05;      //When the difference between the actual speed and targeted speed is smaller than this percentage, the state will display as RUNNNING_NEAR_TARGET.
    private final double CLOSE_ERROR = 0.06;
    public double currentErrorLeft, currentErrorRight;
    public static double KP = 0.11;     //Proportional error constant to tune
    public static double KI = 0;
    public static double KD = 0.001;
    public volatile boolean PID_Modify = false;

    double voltageRatio;
    public static double lowPow = 0.68;
    public static double highPow = 0.75;
    private double targetEncoderRate = 0;
    private int lastEncoderReadingLeft = 0;
    private int lastEncoderReadingRight = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    public volatile int count = 1;


    public static int interval = 500;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public enum FlywheelState {
        STATE_STOPPED, STATE_ACCELERATING, STATE_ADJUSTING, STATE_RUNNING_NEAR_TARGET
    }


    public FlywheelTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        flywheelRight = opMode.hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = opMode.hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void run() {
        double totalErrorRight = 0;
        double totalErrorLeft = 0;
        double prevErrorR = 0;
        double prevErrorL = 0;
        timer.reset();
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(4);
        while(opMode.opModeIsActive() && running) {
            if (teleOp) {
                if (opMode.gamepad2.a) {
                    setFlywheelPow(lowPow);
                } else if (opMode.gamepad2.x) {
                    setFlywheelPow(0);
                } else if (opMode.gamepad2.b) {
                    setFlywheelPow(highPow);
                } else if (opMode.gamepad2.y) {
                    setFlywheelPow(-0.4);
                }
            }
            df.setMaximumFractionDigits(3);



            if(state == FlywheelState.STATE_ACCELERATING) {
                if(timer.time() > 1000) {//Give the flywheel 1.2 seconds to power up before adjusting speed
                    state = FlywheelState.STATE_ADJUSTING;
                }
            } else if (state == FlywheelState.STATE_ADJUSTING || state == FlywheelState.STATE_RUNNING_NEAR_TARGET) {
                int encoderReadingLeft = getEncoderLeft();
                int encoderReadingRight = getEncoderRight();
                if(lastEncoderReadingLeft == 0 && lastEncoderReadingRight == 0) {
                    //We just entered this state from the adjusting state, update encoder and time values.
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                    timer.reset();
                } else if (timer.time() > interval) {
                    double approxRateLeft =  (encoderReadingLeft - lastEncoderReadingLeft)*1.0/timer.time();
                    double approxRateRight =  (encoderReadingRight - lastEncoderReadingRight)*1.0/timer.time();
                    currentErrorLeft = (approxRateLeft - targetEncoderRate)/targetEncoderRate;
                    currentErrorRight = (approxRateRight - targetEncoderRate)/targetEncoderRate;
                    if(Math.abs(currentErrorLeft) < MAX_ALLOWED_ERROR && Math.abs(currentErrorRight) < MAX_ALLOWED_ERROR) {
                        state = FlywheelState.STATE_RUNNING_NEAR_TARGET;
                    } else {
                        state = FlywheelState.STATE_ADJUSTING;
                    }
                    double errorRight = targetEncoderRate - approxRateRight;
                    double errorLeft = targetEncoderRate - approxRateLeft;
                    totalErrorRight += errorRight;
                    totalErrorLeft += errorLeft;
                    double PR = errorRight * KP * voltageRatio;
                    double IR = totalErrorRight * KI * voltageRatio;
                    double DR = (errorRight - prevErrorR) * KD * voltageRatio;
                    double PL = errorLeft * KP * voltageRatio;
                    double IL = totalErrorLeft * KI * voltageRatio;
                    double DL = (errorLeft - prevErrorL) * KD * voltageRatio;
                    rightPower += PR + IR + DR;
                    leftPower += PL + IL + DL;
                    leftPower = range(leftPower);
                    rightPower = range(rightPower);
                    System.out.println();
                    //System.out.println("Target: " + df.format(targetEncoderRate) + " Left rate: " + df.format(approxRateLeft) + " Right rate: " + df.format(approxRateRight));
                    count++;
                    System.out.println(count + ". Left Error: " + df.format(currentErrorLeft * 100) + " Right Error: " + df.format(currentErrorRight * 100));
                    //System.out.println("LPower " + df.format(leftPower) + " RPower " + df.format(rightPower));
                    opMode.telemetry.addData("Left Error(%)", df.format(currentErrorLeft*100));
                    opMode.telemetry.addData("Right Error(%)", df.format(currentErrorRight*100));
                    opMode.telemetry.addData("Left Power", flywheelLeft.getPower());
                    opMode.telemetry.addData("Right Power", flywheelRight.getPower());
                    opMode.telemetry.addData("State", getFlywheelState());
                    opMode.telemetry.update();
                    //if (PID_Modify) {
                        updatePowers();
                    //}
                    timer.reset();
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                    prevErrorR = errorRight;
                    prevErrorL = errorLeft;
                }

            }

        }
        flywheelRight.setPower(0);
        flywheelLeft.setPower(0);
    }

    private double range(double rawPower) {
        if(rawPower > 1) {
            System.out.println("Raw power going over 1, tune your code better");
            return 1;
        } else if (rawPower < -1) {
            System.out.println("Raw power going under 1, something is seriously wrong");
            return -1;
        } else {
            return rawPower;
        }
    }

    public void setFlywheelPow(double power) {
        setFlywheelPow(power, true);
    }

    public void setFlywheelPow(double power, boolean setPow) {
        PID_Modify = true;
        if (timer2.time() > 200) {
            System.out.println("Changed Power " + power);
            if (power == 0) {
                state = state.STATE_STOPPED;
            } else {
                state = state.STATE_ACCELERATING;
            }
            timer.reset();
            voltageRatio = EXPECTED_VOLTAGE / voltage;
            targetEncoderRate = (MAX_ENCODER_TICKS_PER_MS * power);
            if (setPow) {
                leftPower = rightPower = power * MAXPOWER * voltageRatio;
                updatePowers();
            }
            //We intentionally set the power so that it is highly likely to be lower than the
            //"correct" value so that it continues to adjust upwards.
            lastEncoderReadingRight = 0;
            lastEncoderReadingLeft = 0;
            timer2.reset();
        }
    }

    private void updatePowers() {
        flywheelRight.setPower(rightPower);
        flywheelLeft.setPower(leftPower);
    }

    private int getEncoderLeft() {
        return flywheelLeft.getCurrentPosition();
    }

    private int getEncoderRight() {
        return flywheelRight.getCurrentPosition();
    }

    public FlywheelState getFlywheelState() { return state; }

    public String getFlywheelStateString() {
        return state.toString();
    }

}