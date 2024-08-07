package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static java.lang.Math.abs;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.control.EricPid;

@Config
public class Extendo {
    public enum ExtendoState {
        MANUAL,
        IDLE,
        FIRST_THRESHOLD, //MIN
        SECOND_THREESHOLD, //MID
        THIRD_THREESHOLD, //MAX
    }

    CachingDcMotorEx eMotor;
    Sensors sensors;
    PriorityMotor extendoMotor;

    public static double extendoPower = 0.0;
    int indexManualPowers = 0;

    public static double kP = 0, kI = 0, kD = 0;

    double[] distancesThreeshold = new double[]{0, 0, 0}; //TODO: TUNE
    public static double lenght = 0;

    Robot2 robot;
    double vel = 0;
    double power = 0;

    public static double targetLength = 0;

    public static double kg = 0;
    public static double ticksToInches = 0.0; //TUNE
    public static double maxSlidesHeight = 27.891; //TUNE
    final EricPid pid;

    public static double targetPosition = 0;
    public ExtendoState extendoState = ExtendoState.IDLE;

    public Extendo(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot2 robot) {
        this.sensors = sensors;
        this.extendoState = ExtendoState.IDLE;
        if (Globals.RUNMODE == Perioada.AUTO || Globals.RUNMODE == Perioada.TELEOP){
            resetSlidesEncoder();
        }
        if(Globals.RUNMODE == Perioada.TELEOP) {
            extendoState = ExtendoState.MANUAL;
        }
        eMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"));

        extendoMotor = new PriorityMotor(
                eMotor,
                "extendo",
                2
        );
        hardwareQueue.addDevice(extendoMotor);
        this.robot = robot;
        pid = new EricPid(kP, kI, kD);
    }

    void resetSlidesEncoder() {
        eMotor.setPower(0);
//        eMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        eMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //eMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        eMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition = 0;

        eMotor.setPower(0);
    }

    public void updatePower(double Power) {
        extendoPower = Power;
        if(extendoPower == 0) extendoPower = 0.01;
    }

    public void AutoUpdate() {
        lenght = sensors.getExtendoPos()* ticksToInches; //TODO:USE the sensor class for this
        vel = sensors.getExtendoVelocity() * ticksToInches;
        if (abs(lenght - targetLength) < 0.5) {
            extendoState = ExtendoState.IDLE;
        }
        power = pid.update(lenght) + kg;

        extendoMotor.setTargetPower(power);
    }

    public void update() {
        lenght = sensors.getExtendoPos()* ticksToInches; //TODO:USE the sensor class for this
        vel = sensors.getExtendoVelocity() * ticksToInches;
        switch (extendoState) {
            case MANUAL:
                extendoMotor.setTargetPower(extendoPower);
                break;
            case IDLE:
                extendoMotor.setTargetPower(0);
                break;
            case FIRST_THRESHOLD:
                setTargetLength(distancesThreeshold[0]);
                AutoUpdate();
                break;
            case SECOND_THREESHOLD:
                setTargetLength(distancesThreeshold[1]);
                AutoUpdate();
                break;
            case THIRD_THREESHOLD:
                setTargetLength(distancesThreeshold[2]);
                AutoUpdate();
                break;
        }
    }

    public void setTargetLength(double targetLength) {
        this.targetLength = Math.max(Math.min(targetLength, maxSlidesHeight), 0);
        pid.setTarget(targetLength);
    }

    public void forceShutDown() {
        extendoMotor.setPowerForced(0);
    }

    public double getLength() {
        return lenght;
    }
}
