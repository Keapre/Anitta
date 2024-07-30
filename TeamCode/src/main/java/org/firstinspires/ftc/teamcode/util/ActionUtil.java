package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;

import java.util.concurrent.Callable;

public class ActionUtil {
    public static class DcMotorExPowerAction implements Action {
        double power;
        PriorityMotor motor;

        public DcMotorExPowerAction(PriorityMotor motor, double power) {
            this.power = power;
            this.motor = motor;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            motor.motor[0].setPower(power);
            return false;
        }
    }

    public static class RunnableAction implements Action {
        Callable<Boolean> action;

        public RunnableAction(Callable<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                return action.call();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class ServoPositionAction implements Action {
        double position;
        PriorityServo servo;

        public ServoPositionAction(PriorityServo servo, double position) {
            this.servo = servo;
            this.position = position;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            servo.servo[0].setPosition(position);
            return false;
        }
    }
}