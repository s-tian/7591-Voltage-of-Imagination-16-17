package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class DriveTrainTask extends Thread {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private LinearOpMode opMode;

    public volatile boolean running = true;

    public DriveTrainTask(LinearOpMode opMode, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft  = backLeft;
        this.backRight = backRight;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            double joy1Y = -opMode.gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = opMode.gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = opMode.gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            if(Math.abs(joy1Y)==0 && Math.abs(joy1X)==0 && Math.abs(joy2X)==0){
                joy2X = 0.3*(opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger);
                if(joy2X==0) {
                    joy2X = 0.3 * (opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger);
                }
                joy1Y = -opMode.gamepad2.left_stick_y;
                joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*0.3: 0;
                joy1X = opMode.gamepad2.left_stick_x;
                joy1X = Math.abs(joy1X) > 0.15 ? joy1X*0.3: 0;
                if(joy1X==0 && joy1Y==0){
                    if(opMode.gamepad1.left_bumper){
                        joy1X = -0.3;
                    }
                    if(opMode.gamepad1.right_bumper){
                        joy1X = 0.3;
                    }
                }
            }
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
