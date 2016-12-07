package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;

/**
 * Created by Howard on 10/15/16.
 */
public class IntakeTask extends Thread {

    private ThreadedTeleOp opMode;
    public volatile boolean running = true;
    private VOISweeper sweeper;


    public IntakeTask(ThreadedTeleOp opMode, VOISweeper sweeper) {
        this.sweeper = sweeper;
        this.opMode = opMode;
        sweeper.setPower(0);
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if(opMode.gamepad1.y){
                sweeper.setPower(1);
            }
            else if(opMode.gamepad1.b){
                sweeper.setPower(0);
            }
            else if(opMode.gamepad1.a){
                sweeper.setPower(-1);
            }

        }
        sweeper.setPower(0);
    }
}
