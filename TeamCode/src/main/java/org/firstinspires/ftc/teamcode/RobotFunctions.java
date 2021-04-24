package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.io.Serializable;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RobotFunctions {
    float Red;
    float Green;
    float Blue;
    double counter;
    double RedFactor;
    double GreenFactor;
    double Blue_Factor;
    boolean FunctionDone;
    int mm_tick;
    int mm_in;
    RobotHardware robot = new RobotHardware();
    public void Telemetries() {
        telemetry.addData(String.valueOf(robot.leftDrive), "Did this work?");
    }
    public void Move(int in, boolean forward_, boolean moveoverride) {
        FunctionDone = false;
        while (!FunctionDone) {
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
                FunctionDone = true;
            }
        }
    }
    public boolean CheckColour(String Hex,String Colour) {
        boolean ColourCheck;
        int i;
        double R;
        double G;
        double B;
        List<Serializable> HexList = null;

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
                        HexList.add(JavaUtil.inTextGetLetter(Hex, JavaUtil.AtMode.FROM_START, (i - 1)));
                        break;
                }
            }
            R = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 0, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 1, false);
            G = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 2, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 3, false);
            B = (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 4, false) * 16 + (Double) JavaUtil.inListGet(HexList, JavaUtil.AtMode.FROM_START, 5, false);
            Red = Math.round(robot.color1.red() * (255 / RedFactor));
            Green = Math.round(robot.color1.green() * (255 / GreenFactor));
            Blue = Math.round(robot.color1.blue() * (255 / Blue_Factor));
            ColourCheck = Within_Range(10, R, Red) && Within_Range(10, G, Green) && Within_Range(10, B, Blue);
        } else {
            ColourCheck = true;
        }
        return ColourCheck;
    }
}
