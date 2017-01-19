package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;

import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor capBottom, capTop;
    private Servo forkLeft, forkRight;
    private LinearOpMode opMode;
    double startLeft = 0.55, startRight = 0.12, downLeft = 0.00, downRight = 0.62; // fork lift positions
    boolean aPushed = false;
    public volatile boolean running = true;
    boolean forkliftOut = false;
    double voltage = 12.5;
    double expectedVoltage = 12.5;
    double targetPower = 0;
    public static double holdPower = 0.016;
    public static double editPower = 0.0002;

    int targetPosition = 0;
    int bottomPosition = 0;
    int topPosition = 0;
    double KP = 0.015;
    int delta = 75;

    boolean changedMode = false;
    int beforeEnd = 10000;
    boolean pressing = false;
    public static final double flStart = 0.55, frStart = 0.13; // fork left initialize positions
    // decrease flStart and increase frStart to make forklift more out
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer3 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public CapBallTask(LinearOpMode opMode) {
        this.opMode = opMode;

        forkLeft = opMode.hardwareMap.servo.get("forkLeft");
        forkRight = opMode.hardwareMap.servo.get("forkRight");

        capBottom = opMode.hardwareMap.dcMotor.get("capBottom");
        capTop = opMode.hardwareMap.dcMotor.get("capTop");
        capBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        capTop.setDirection((DcMotorSimple.Direction.REVERSE));
        bottomPosition = capBottom.getCurrentPosition();
        topPosition = capTop.getCurrentPosition();
        double mc7 = opMode.hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = opMode.hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = opMode.hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = opMode.hardwareMap.voltageSensor.get("flywheels").getVoltage();
        voltage = (mc7 + mc6 + mc3 + mc2) / 4;
        setForkPosition();
        setLiftPower(0);

    }

    @Override
    public void run() {
        timer2.reset();
        while(opMode.opModeIsActive() && running) {

            if (opMode.gamepad1.right_bumper || opMode.gamepad2.right_trigger > 0.15) {
                setLiftPower(1);
            } else if (opMode.gamepad1.left_bumper || opMode.gamepad2.left_trigger > 0.15) {
                setLiftPower(-0.2);
            } else if (Math.abs(opMode.gamepad2.right_stick_x) > 0.15 || Math.abs(opMode.gamepad2.right_stick_y) > 0.15){
                holdPosition();
            } else {
                setLiftPower(0);
            }


            if (opMode.gamepad1.dpad_down && !forkliftOut) {
                forkliftOut = true;
                setLiftPower(1);
                while(opMode.opModeIsActive() && timer.time() < 600);
                setLiftPower(0);
                while(opMode.opModeIsActive() && timer.time() < 500);
                setLiftPower(-1);
                while(opMode.opModeIsActive() && timer.time() < 600);
                setLiftPower(0);
                forkliftOut = false;
                pressing = true;
            }

            if(opMode.gamepad2.right_bumper) {
                forkLeft.setPosition(downLeft);
                forkRight.setPosition(downRight);
                forkliftOut = false;
            } else {
                forkLeft.setPosition(startLeft);
                forkRight.setPosition(startRight);
            }

        }
        setLiftPower(0);
    }

    public void setLiftPower(double power){
        capBottom.setPower(power);
        capTop.setPower(power);
    }

    public void setForkPosition() {
        forkLeft.setPosition(startLeft);
        forkRight.setPosition(startRight);
    }
    public void setMode(DcMotor.RunMode mode) {
        if(capTop.getMode() != mode) {
            capBottom.setMode(mode);
            capTop.setMode(mode);
        }
    }

    public void setPosition(int pos) {
        capBottom.setTargetPosition(pos);
        capTop.setTargetPosition(pos);
    }
    public void holdPosition() {
        topPosition = capTop.getCurrentPosition();
        bottomPosition = capBottom.getCurrentPosition();
        int range = 3;
        setLiftPower(holdPower * expectedVoltage / voltage);
        timer3.reset();
        while (Math.abs(opMode.gamepad2.right_stick_x) > 0.15 || Math.abs(opMode.gamepad2.right_stick_y) > 0.15 && opMode.opModeIsActive()) {
            if (timer3.time() > 50) {
                if (capTop.getCurrentPosition() < topPosition - range && capBottom.getCurrentPosition() < bottomPosition - range) {
                    holdPower += editPower;
                    setLiftPower(holdPower * expectedVoltage / voltage);
                }
                if (capTop.getCurrentPosition() > topPosition + range && capBottom.getCurrentPosition() > bottomPosition + range) {
                    holdPower -= editPower;
                    setLiftPower(holdPower * expectedVoltage / voltage);
                }
                timer3.reset();

            }
        }
        setLiftPower(0);
    }




//    private void dropForklift() {
//        //Probably want to use encoders for this
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        timer.reset();
//        while(timer.time() < 1400 && opMode.opModeIsActive()) {
//            if(opMode.gamepad1.right_bumper||opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger>0.15 || opMode.gamepad1.left_bumper||opMode.gamepad2.left_trigger-opMode.gamepad2.right_trigger>0.15) {
//                return;
//            }
//            if(timer.time() < 500) {
//                setLiftPower(1);
//            } else {
//                setLiftPower(-1);
//            }
//        }
//        setLiftPower(0);
//        forkliftOut = true;
//    }

}