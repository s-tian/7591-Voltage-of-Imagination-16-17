package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends TaskThread {

    private DcMotor capBottom, capTop;
    private Servo forkLeft, forkRight;
    double startLeft = 0.55, startRight = 0.12, downLeft = 0.00, downRight = 0.62; // fork lift positions
    boolean aPushed = false;
    boolean forkliftOut = false;
    double targetPower = 0;
    public static double holdPower = 0.5;
    public static double editPower = 0.05;

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
        initialize();
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
            } else if (opMode.gamepad2.dpad_right){
                holdPosition();
            } else {
                setLiftPower(0);
            }


            if (opMode.gamepad1.dpad_down && !forkliftOut) {
                forkliftOut = true;
                setLiftPower(1);
                sleep(600);
                setLiftPower(0);
                sleep(500);
                setLiftPower(-1);
                sleep(600);
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

    @Override
    public void initialize()  {
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
    }

    public void setLiftPower(double power) {
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
        setLiftPower(holdPower * EXPECTED_VOLTAGE / voltage);
        timer3.reset();
        while (opMode.gamepad2.dpad_right && opMode.opModeIsActive()) {
            if (opMode.gamepad1.right_bumper || opMode.gamepad1.left_bumper) {
                return;
            }
            if (timer3.time() > 50) {
                if (capTop.getCurrentPosition() < topPosition - range && capBottom.getCurrentPosition() < bottomPosition - range) {
                    //holdPower += editPower;
                    setLiftPower(holdPower * EXPECTED_VOLTAGE / voltage);
                }
                if (capTop.getCurrentPosition() > topPosition + range && capBottom.getCurrentPosition() > bottomPosition + range) {
                    //holdPower -= editPower;
                    setLiftPower(holdPower * EXPECTED_VOLTAGE / voltage);
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