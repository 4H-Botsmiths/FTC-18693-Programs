package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RobotFunctions {
    RobotHardware robot = new RobotHardware();

    public void Telementrys() {
        telemetry.addData(String.valueOf(robot.LeftDrive), "Did this work?");
    }


}
