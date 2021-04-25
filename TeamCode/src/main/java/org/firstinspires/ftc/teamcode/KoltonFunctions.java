package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class KoltonFunctions {
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void Telemetries() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors Velocity", "left (%.2f), right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.update();
    }
    public void Ramp() {
        if (-gamepad2.right_stick_y > 0) {

        } else if (-gamepad2.right_stick_y < 0) {

        }
    }
}