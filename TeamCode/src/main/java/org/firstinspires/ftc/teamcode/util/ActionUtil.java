package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;

import java.util.concurrent.Callable;

public class ActionUtil {
    public static class DcMotorExPowerAction implements Action {
        double power;
        CachingDcMotorEx motor;

        public DcMotorExPowerAction(CachingDcMotorEx motor, double power) {
            this.power = power;
            this.motor = motor;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            motor.setPower(power);
            return false;
        }
    }

//    public static class RunnableAction implements Action {
//        Callable<Boolean> action;
//
//        public RunnableAction(boolean action) {
//            this.action = action;
//        }
//
//        @Override
//        public boolean run(TelemetryPacket packet) {
//            try {
//                return action.call();
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//        }
//    }

    public static class ServoPositionAction implements Action {
        double position;
        CachingServo servo;

        public ServoPositionAction(CachingServo servo, double position) {
            this.servo = servo;
            this.position = position;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            servo.setPosition(position);
            return false;
        }
    }
}