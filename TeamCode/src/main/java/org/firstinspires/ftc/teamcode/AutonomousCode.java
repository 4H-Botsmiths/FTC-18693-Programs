package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous", group = "ForRobot")
public class AutonomousCode extends LinearOpMode {
    WilliamFunctions functions = new WilliamFunctions();

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Please wait");
        telemetry.update();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            if (functions.CheckColour("FFFFFF", ""))
            functions.Move(48, true, false);
            functions.Turn(90, false);
            functions.Move(24, true, false);
            functions.Turn(-90, false);
            while (opModeIsActive()) {
                functions.Telemetries();
            }
        }
    }
}

