package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.Serializable;
import java.util.List;

@Autonomous(name = "Autonomous", group = "ForRobot")
public class AutonomousCode extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    WilliamFunctions functions = new WilliamFunctions();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Please wait");
        telemetry.update();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
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

