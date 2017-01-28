package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Shooter Test", group = "Test")

public class ShooterTest extends LinearOpMode {

    FlywheelTask flywheelTask;
    IntakeTask intakeTask;


    @Override
    public void runOpMode() throws InterruptedException {
        flywheelTask = new FlywheelTask(this);
        intakeTask = new IntakeTask(this);
        TaskThread.calculateVoltage(this);
        flywheelTask.teleOp = intakeTask.teleOp = true;
        setPowers();
        waitForStart();
        flywheelTask.start();
        intakeTask.start();
        while(opModeIsActive());
    }

    public void setPowers() {
        boolean confirmed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        while (!confirmed) {
            if (gamepad2.a && !aPressed) {
                aPressed = true;
                FlywheelTask.lowPow += 0.01;
            }
            if (gamepad2.b && !bPressed) {
                bPressed = true;
                FlywheelTask.lowPow -= 0.01;
            }
            if (gamepad2.x && !xPressed) {
                xPressed = true;
                FlywheelTask.highPow += 0.01;
            }
            if (gamepad2.y && !yPressed) {
                yPressed = true;
                FlywheelTask.highPow -= 0.01;
            }

            if (!gamepad2.a){
                aPressed = false;
            }
            if (!gamepad2.b) {
                bPressed = false;
            }
            if (!gamepad2.x){
                xPressed = false;
            }
            if (!gamepad2.y){
                yPressed = false;
            }
            telemetry.addData("A (low)", FlywheelTask.lowPow);
            telemetry.addData("B (high)", FlywheelTask.highPow);
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();


        }
    }
}
