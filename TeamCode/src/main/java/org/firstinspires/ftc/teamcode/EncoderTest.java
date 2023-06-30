package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "mrbean")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        ElapsedTime timer = new ElapsedTime();
        double last = 0, cur = 0;
        timer.reset();
        robot.encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            last = cur;
            cur = timer.milliseconds();
            telemetry.addData("encoder pos", robot.encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}