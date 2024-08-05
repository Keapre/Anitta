package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;

public class BetaDrive {
    private boolean slow_mode = false;
    private static double turnSpeed = 0.5;
    private static double slow_turnSpeed = 0.3;
    private static double slow_driving = 0.6;

    CachingDcMotorEx frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    PriorityMotor PfrontLeftMotor,PbackLeftMotor,PfrontRightMotor,PbackRightMotor;
    public BetaDrive(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        frontLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"leftFront"));
        backLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"leftBack"));
        frontRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"rightFront"));
        backRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"rightBack"));

        PfrontLeftMotor = new PriorityMotor(
                frontLeftMotor,
                "frontLeft",
                4
        );
        PbackLeftMotor = new PriorityMotor(
                backLeftMotor,
                "backLeft",
                4
        );
        PfrontRightMotor = new PriorityMotor(
                frontRightMotor,
                "frontRight",
                4
        );
        PbackRightMotor = new PriorityMotor(
                backRightMotor,
                "backRight",
                4
        );
        PbackLeftMotor.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        PfrontLeftMotor.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        PbackRightMotor.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);

        hardwareQueue.addDevice(PbackLeftMotor);
        hardwareQueue.addDevice(PfrontRightMotor);
        hardwareQueue.addDevice(PfrontLeftMotor);
        hardwareQueue.addDevice(PbackRightMotor);
    }

    public void driveGamepad(GamePadController g1) {

        if(g1.leftStickButtonOnce()){
            slow_mode = !slow_mode;
        }


        double y = -g1.left_stick_y;
        double x = g1.left_stick_x * 1.1;
        double rx = g1.right_stick_x;

        if(slow_mode) {
            y*=slow_driving;
            x*=slow_driving;
            rx*=slow_turnSpeed;
        }else {
            rx*=turnSpeed;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        PfrontLeftMotor.setTargetPower(frontLeftPower);
        PbackLeftMotor.setTargetPower(backLeftPower);
        PfrontRightMotor.setTargetPower(frontRightPower);
        PbackRightMotor.setTargetPower(backRightPower);
    }


}
