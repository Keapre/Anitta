package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.control.EricPid;

@Config
public class Slides2 {

    Robot3 robot;

    public enum SlidesState {
        MANUAL,

        IDLE,
        FIRST_THRESHOLD, //MIN
        SECOND_THREESHOLD, //MID
        THIRD_THREESHOLD, //MAX
    }

    public CachingDcMotorEx sMotor1, sMotor2;
    Sensors sensors;

    double[] manualPowers = new double[]{0.7, -0.7, 0.0};
    int indexManualPowers = 0;

    public static double idlePower = 0.05;
    public SlidesUpdate slidesUpdate;
    public static double extendPower = 0.8;
    public static double retractPower = -0.8;
    public static double kP = 0, kI = 0, kD = 0;
    public static double kgSlides = 0;

    public static double ExtendoPower = 0;

    double[] distancesThreeshold = new double[]{0, 0, 0}; //TODO: TUNE
    public static double lenght = 0;

    double vel = 0;
    double power = 0;

    public static double targetLength = 0;
    public static double ticksToInches = 0.0; //TUNE
    public static double maxSlidesHeight = 27.891; //TUNE
    final EricPid pid;
    public static double slidePower = 0;

    public static double targetPosition = 0;
    public SlidesState slidesState = SlidesState.MANUAL;

    public Slides2(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot3 robot) {
        slidesUpdate = new SlidesUpdate(robot);
        this.sensors = sensors;
        this.slidesState = SlidesState.MANUAL;

        sMotor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "outtake1")); //2-are encoder
        sMotor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "outtake2"));

        resetSlidesEncoder();
        pid = new EricPid(kP, kI, kD);
        this.robot = robot;
    }

    void resetSlidesEncoder() {
        sMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        sMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        sMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sMotor1.setPower(0);
        sMotor2.setPower(0);
    }


    public void updatePower(double power) {
        slidePower = power;
    }
    public void firstThreeshold() {
        slidesState = SlidesState.FIRST_THRESHOLD;
        setTargetLength(distancesThreeshold[0]);
    }

    public void secondThreeshold() {
        slidesState = SlidesState.SECOND_THREESHOLD;
        setTargetLength(distancesThreeshold[1]);
    }


    public void thirdThreeshold() {
        slidesState = SlidesState.THIRD_THREESHOLD;
        setTargetLength(distancesThreeshold[2]);
    }



    public double getLenght() {
        return sensors.getSlidePos() * ticksToInches;
    }

    public double getVelocity() {
        return sensors.getSlideVelocity() * ticksToInches;
    }

    public void AutoUpdate() { // no feedforward
        lenght = getLength();
        vel = getVelocity();
        if (abs(lenght - targetLength) < 0.5) {
            slidesState = SlidesState.IDLE;
        }
        power = pid.update(lenght) + kgSlides;
        sMotor2.setPower(power);
        sMotor1.setPower(power);
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        this.pid.target = targetPosition;
    }

    private class TargetPosition implements Action {

        double position;

        public TargetPosition(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (getTargetPosition() != position) {
                setTargetPosition(position);
            }
            return false;
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }
/*
    public void feedforward() {
        double error = targetLength - lenght;

        if(targetLength)
    }
*/

    //    public void checkForIntake() {
//        if(robot.outtake.currentState == Outtake.FourBarState.TRANSFER_INTAKE) {
//            firstThreeshold();
//        }else if(robot.outtake.currentState == Outtake.FourBarState.OUTTAKE_POSITION) {
//            secondThreeshold();
//        }
//    }
    public void update() {
        switch (slidesState) {
            case MANUAL:
                sMotor2.setPower(slidePower);
                sMotor1.setPower(slidePower);
                break;
            case FIRST_THRESHOLD:
                firstThreeshold();
                AutoUpdate();
                break;
            case SECOND_THREESHOLD:
                secondThreeshold();
                AutoUpdate();
                break;
            case THIRD_THREESHOLD:
                thirdThreeshold();
                AutoUpdate();
                break;
        }
    }

    public void setTargetLength(double targetLength) {
        targetPosition = Math.max(Math.min(targetLength, maxSlidesHeight), 0);
        pid.setTarget(targetPosition);
    }

    public void forceShutDown() {

        sMotor1.setPower(0);
        sMotor2.setPower(0);
    }

    public double getLength() {
        return lenght;
    }


    private class changeSlideState implements Action {

        SlidesState state;

        public changeSlideState(SlidesState state) {
            this.state = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slidesState = state;
            return false;
        }
    }

    public Action changeState(SlidesState state) {
        return new changeSlideState(state);
    }
}
