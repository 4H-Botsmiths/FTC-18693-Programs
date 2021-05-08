package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous", group = "ForRobot")
public class AutonomousCode extends LinearOpMode {
    WilliamFunctions functions = new WilliamFunctions();
   // RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Please wait");
        telemetry.update();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        functions.runOpMode();
        waitForStart();
        if (opModeIsActive()) {
            //if (functions.CheckColour("FFFFFF", "")) {
                functions.Move(48, true, false);
                functions.Turn(90, false);
                functions.Move(24, true, false);
                functions.Turn(-90, false);
           // }
            while (opModeIsActive()) {
                functions.Telemetries();
            }
        }
    }
}


