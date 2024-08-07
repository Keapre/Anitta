package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.google.ftcresearch.tfod.util.ImageUtils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;

public class BetaDrive {
    private boolean slow_mode = false;
    private static double turnSpeed = 0.5;
    private static double slow_turnSpeed = 0.3;
    IMU imu;
    private static double slow_driving = 0.6;
    private static double drivingSpeed = 0.8;

    IMU.Parameters imuPar = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );
    CachingDcMotorEx frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    public BetaDrive(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        imu = hardwareMap.get(IMU.class,"imu");
        frontLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"leftFront"));
        backLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"leftBack"));
        frontRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"rightFront"));
        backRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"rightBack"));
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(imuPar);
    }


    public void driveGamepad(GamePadController g1) {

        if(g1.leftStickButtonOnce()){
            slow_mode = !slow_mode;
        }


        if(g1.rightStickButtonOnce()) {
            imu.resetYaw();
        }


        double y = -g1.left_stick_y;
        double x = g1.left_stick_x;
        double rx = g1.right_stick_x;

        if(slow_mode) {
            y*=slow_driving;
            x*=slow_driving;
            rx*=slow_turnSpeed;
        }else {
            rx*=turnSpeed;
            y*=drivingSpeed;
            x*=drivingSpeed;
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }


}