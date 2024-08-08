package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static java.lang.Math.abs;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.SlidesUpdate;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.util.control.EricPid;

@Config
public class Extendo2 {
    public enum ExtendoState {
        EXTEND,
        RETRACT,
        IDLE,
        FIRST_THRESHOLD, //MIN
        SECOND_THREESHOLD, //MID
        THIRD_THREESHOLD, //MAX
    }

    CachingDcMotorEx eMotor;
    Sensors sensors;
    public static double retractPower = -1;
    public static double extendPower = 1;
    public static double idlePower = 1;


    public static double extendoPower = 0.0;
    int indexManualPowers = 0;

    public static double kP = 0, kI = 0, kD = 0;

    double[] distancesThreeshold = new double[]{0, 0, 0}; //TODO: TUNE
    public static double lenght = 0;

    Robot3 robot;
    double vel = 0;
    double power = 0;

    public static double targetLength = 0;

    public static double kg = 0;
    public static double ExtendoPower = 0;
    public static double ticksToInches = 0.0; //TUNE
    public static double maxSlidesHeight = 27.891; //TUNE
    final EricPid pid;
    public ExtendoUpdate update;

    public static double targetPosition = 0;
    public ExtendoState extendoState = ExtendoState.IDLE;

    public Extendo2(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot3 robot) {
        this.sensors = sensors;
        this.extendoState = ExtendoState.IDLE;
        update = new ExtendoUpdate(robot);

        if(Globals.RUNMODE == Perioada.TELEOP) {
            extendoState = ExtendoState.IDLE;
        }
        eMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"));

        resetSlidesEncoder();
        this.robot = robot;
        pid = new EricPid(kP, kI, kD);
    }
    public double getPower() {
        return extendoPower;
    }
    void resetSlidesEncoder() {
        eMotor.setPower(0);
//        extendoMotor.motor[0].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        eMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        eMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition = 0;

    }

    public void updatePower(double Power) {
        this.extendoPower = Power;
    }

    public void AutoUpdate() {
        lenght = sensors.getExtendoPos()* ticksToInches; //TODO:USE the sensor class for this
        vel = sensors.getExtendoVelocity() * ticksToInches;
        if (abs(lenght - targetLength) < 0.5) {
            extendoState = ExtendoState.IDLE;
        }
        power = pid.update(lenght) + kg;

        eMotor.setPower(power);
    }
    public void updateManualPower(GamePadController g1) {

        if (g1.rightBumperOnce()) {
            if (ExtendoPower == retractPower) {
                ExtendoPower = idlePower;
            } else {
                ExtendoPower = retractPower;
            }

        }
        if (g1.leftBumperOnce()) {
            if (ExtendoPower == extendPower) {
                ExtendoPower = idlePower;
            } else {
                ExtendoPower = extendPower;
            }
        }
    }
    public void update() {
        lenght = sensors.getExtendoPos()* ticksToInches; //TODO:USE the sensor class for this
        vel = sensors.getExtendoVelocity() * ticksToInches;
        switch (extendoState) {
            case EXTEND:
                eMotor.setPower(1);
                break;
            case RETRACT:
                eMotor.setPower(retractPower);
                break;
            case IDLE:
                eMotor.setPower(0.01);
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
        eMotor.setPower(0);
    }

    public double getLength() {
        return lenght;
    }
}
