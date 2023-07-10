package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous (name = "toggletimertest")
public class Timertest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    int Toggletimer = 0;
    waitForStart();
    while (opModeIsActive()){
        Toggletimer +=1;
        if (Toggletimer > 100){
            telemetry.addLine("100");
        }
        if (Toggletimer > 400){
            telemetry.addLine("400");
        }
        if (Toggletimer > 1000){
            telemetry.addLine("1000");
        }
        telemetry.addData("Toggletimer", Toggletimer);
        telemetry.update();
    }
    }
}
