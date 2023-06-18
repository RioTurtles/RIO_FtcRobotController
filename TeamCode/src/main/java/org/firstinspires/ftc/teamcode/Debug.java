package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Debug {
    static void control(String mode, Project1Hardware robot, Telemetry telemetry, Gamepad gamepad1){
        double bucketAPos = robot.bucketAngle.getPosition();
        double clawPos = robot.claw.getPosition();
        double clawAPos = robot.clawAngle.getPosition();

        switch (mode) {
            case "BUCKETANGLE":
                if (gamepad1.left_bumper) {
                    robot.bucketAngle.setPosition(bucketAPos + 0.01);
                    bucketAPos += 0.01;
                } else if (gamepad1.right_bumper) {
                    robot.bucketAngle.setPosition(bucketAPos - 0.01);
                    bucketAPos -= 0.01;
                }
                telemetry.addData("bucket big angle", bucketAPos);
                //0.72 (drop bucket), 0.02
                break;
            case "CLAW":
                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(clawPos + 0.01);
                    clawPos += 0.01;
                } else if (gamepad1.right_bumper) {
                    robot.claw.setPosition(clawPos - 0.01);
                    clawPos -= 0.01;
                }
                telemetry.addData("claw small angle", clawPos);
                break;
            case "CLAW ANGLE":
                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(clawAPos + 0.01);
                    clawAPos += 0.01;
                } else if (gamepad1.right_bumper) {
                    robot.claw.setPosition(clawAPos - 0.01);
                    clawAPos -= 0.01;
                }
                telemetry.addData("claw big angle", clawAPos);
                break;
        }


        //if (gamepad1.dpad_left) dModesIndex = (dModesIndex + 1) % debugModes.length;
        //else dModesIndex = (dModesIndex + 1) % debugModes.length;
        //Debug.control(debugModes[dModesIndex], robot, telemetry, gamepad1);
    }
}
