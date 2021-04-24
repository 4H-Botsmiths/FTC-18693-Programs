package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

@Autonomous(name = "Functions", group = "ForRobot")
@Disabled
public class WilliamFunctions extends LinearOpMode {
    float Red;
    float Green;
    float Blue;
    double counter = 0;
    double RedFactor = 338;
    double GreenFactor = 559;
    double Blue_Factor = 494;
    int Ticks_Rotation = 288;
    int Wheel_Circumference = (int) 282.6;
    int mm_tick = Wheel_Circumference / Ticks_Rotation;
    int mm_in = (int) 25.4;

    RobotHardware robot = new RobotHardware();

    public void Telemetries() {
        telemetry.addData("Motor0 Target Position", robot.leftDrive.getTargetPosition());
        telemetry.addData("Motor1 Target Position", robot.rightDrive.getTargetPosition());
        telemetry.addData("Motor0 Encoder Value", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Motor1 Encoder Value", robot.rightDrive.getCurrentPosition());
        telemetry.addData("Motor0 mm/Second", robot.leftDrive.getVelocity() * mm_tick);
        telemetry.addData("Motor1 mm/Second", robot.rightDrive.getVelocity() * mm_tick);
        telemetry.addData("Motor0 Power", robot.leftDrive.getPower());
        telemetry.addData("Motor1 Power", robot.rightDrive.getPower());
        telemetry.addData("Motor0 Direction", robot.leftDrive.getDirection());
        telemetry.addData("Motor1 Direction", robot.rightDrive.getDirection());
        telemetry.addData("Motor0 Working?", robot.leftDrive.isBusy());
        telemetry.addData("Motor1 Working?", robot.rightDrive.isBusy());
        telemetry.update();
    }

    public void Move(int in, boolean forward_, boolean moveoverride) {
        while (opModeIsActive()) {
            telemetry.addData("Move Waiting", counter);
            Telemetries();
            counter += 1;
            if (forward_) {
                robot.leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
            } else {
                robot.leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
            }
            if (moveoverride || !robot.rightDrive.isBusy()) {
                counter = 0;
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftDrive.setTargetPosition(in * mm_in * mm_tick);
                robot.rightDrive.setTargetPosition(in * mm_in * mm_tick);
                robot.leftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                // set F to 54.1
                robot.leftDrive.setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                robot.rightDrive.setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
            }
        }
    }

    public boolean CheckColour(String Hex, String Colour) {
        boolean ColourCheck;
        int i;
        double R;
        double G;
        double B;
        ArrayList<Integer> HexList = new ArrayList<>();
        if (!Colour.equals("")) {
            switch (Colour) {
                case "Red":
                    Hex = "FF0000";
                    break;
                case "Blue":
                    Hex = "0000FF";
                    break;
                case "White":
                    Hex = "FFFFFF";
                    break;
            }
        }
        if (Hex.length() == 6) {
            for (i = 1; i <= 6; i++) {
                switch (JavaUtil.inTextGetLetter(Hex, JavaUtil.AtMode.FROM_START, (i - 1))) {
                    case "A":
                        HexList.add(10);
                        break;
                    case "B":
                        HexList.add(11);
                        break;
                    case "C":
                        HexList.add(12);
                        break;
                    case "D":
                        HexList.add(13);
                        break;
                    case "E":
                        HexList.add(14);
                        break;
                    case "F":
                        HexList.add(15);
                        break;
                    default:
                        HexList.add(Integer.parseInt(JavaUtil.inTextGetLetter(Hex, JavaUtil.AtMode.FROM_START, (i - 1))));
                        break;
                }
            }
            R = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 0, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 1, false);
            G = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 2, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 3, false);
            B = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 4, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 5, false);
            Update_Color();
            ColourCheck = Within_Range(10, R, Red) && Within_Range(10, G, Green) && Within_Range(10, B, Blue);
        } else {
            ColourCheck = true;
        }
        return ColourCheck;
    }

    public boolean Within_Range(double range, double range_variable, float range_comparison) {
        boolean range_output;
        range_output = range_comparison < range_variable + range && range_comparison > range_variable - range;
        return range_output;
    }

    public void Update_Color() {
        Red = Math.round(robot.color1.red() * (255 / RedFactor));
        Green = Math.round(robot.color1.green() * (255 / GreenFactor));
        Blue = Math.round(robot.color1.blue() * (255 / Blue_Factor));
    }

    public void Turn(double degrees, boolean turnoverride) {
        BNO055IMU.Parameters imuPar;
        Orientation angles = null;

        Telemetries();
        imuPar = new BNO055IMU.Parameters();
        imuPar.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuPar.loggingEnabled = false;
        robot.gyro.initialize(imuPar);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            telemetry.addData("Turn Waiting", counter);
            Telemetries();
            counter += 1;
            if (turnoverride || !robot.rightDrive.isBusy()) {
                degrees = degrees * -1;
                counter = 0;
                if (degrees > 0) {
                    robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                } else {
                    robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
                while (!(Within_Range(1, degrees, angles != null ? angles.firstAngle : 0) || isStopRequested())) {
                    angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("angle", angles.firstAngle);
                    telemetry.addData("target", degrees);
                    telemetry.addData("done?", Within_Range(1, degrees, angles.firstAngle) || isStopRequested());
                    Telemetries();
                }
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            }
        }
    }

    @Override
    public void runOpMode() {

    }
}