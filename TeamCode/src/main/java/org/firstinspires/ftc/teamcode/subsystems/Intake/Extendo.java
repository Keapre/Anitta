package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Perioada;
import org.firstinspires.ftc.teamcode.util.control.EricPid;

@Config
public class Extendo implements Subsystem {
    public enum ExtendoState {
        IDLE(0.07),
        MIN(0), //MIN

        MANUAL(0),
        MAX(2200); //MAX

        public double target;
        ExtendoState(double tg){
            this.target = tg;
        }
    }

    CachingDcMotorEx eMotor;
    Sensors sensors;
    public static double retractPower = -1;
    public static double extendPower = 1;
    public static double idlePower = 1;


    public static double extendoPower = 0.0;
    int indexManualPowers = 0;

    public static double kP = 0.0395,kI = 0.002,kD = 0.001;

    double[] distancesThreeshold = new double[]{0, 2200, 0}; //TODO: TUNE
    public static double encoderValue = 0;

    Robot robot;
    double vel = 0;
    double power = 0;



    public ExtendoState extendoState = ExtendoState.MIN;

    public static double kg = 0;
    public static double ExtendoPower = 0;
    public static double ticksToInches = 0.0; //TUNE
    public static double maxSlidesHeight = 800; //TUNE
    final PIDController pid;
    public static PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);



    public static double targetPosition = 0;
    public int defaultEncoderValue = 0;

    public Extendo(HardwareMap hardwareMap, Sensors sensors, Robot robot) {
        this.sensors = sensors;
        this.pid = new PIDController(coef.p,coef.i,coef.d);
        this.extendoState = ExtendoState.IDLE;
        eMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"));

        resetSlidesEncoder();
        defaultEncoderValue = eMotor.getTargetPosition();
        this.robot = robot;
    }
    public double getPower() {
        return extendoPower;
    }
    public int getEnoder() {
        return eMotor.getCurrentPosition() - defaultEncoderValue;
    }
    void resetSlidesEncoder() {
        eMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        eMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void hangMode() {
        eMotor.close();
    }
    public void updatePower(double Power) {
        extendoState = ExtendoState.MANUAL;
        extendoPower = Power;
        if(Power == 0) {
            extendoState = ExtendoState.IDLE;
        }
    }


    void autoUpdate() {
        pid.setSetPoint(extendoState.target);
        encoderValue = eMotor.getCurrentPosition();
        double power = pid.calculate(encoderValue);
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

        switch (extendoState) {
            case IDLE:
                eMotor.setPower(-extendoState.target);
                break;
            case MANUAL:
                eMotor.setPower(extendoPower);
                break;
            case MIN:
                setTargetLength(distancesThreeshold[0]);
                autoUpdate();
                break;
            case MAX:
                setTargetLength(distancesThreeshold[1]);
                autoUpdate();
                break;
        }
    }

    public void setTargetLength(double targetLength) {
        pid.setSetPoint(targetLength);
    }

    public void forceShutDown() {
        eMotor.setPower(0);
    }

}
