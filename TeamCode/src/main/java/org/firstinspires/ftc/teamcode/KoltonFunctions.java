package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class KoltonFunctions {
    RobotHardware robot = new RobotHardware();

    public void Telemetries() {
        telemetry.update();
    }
}