package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
public class Slides {

    Robot2 robot;
    public enum SlidesState {
        MANUAL,
        IDLE,
        FIRST_THRESHOLD, //MIN
        SECOND_THREESHOLD, //MID
        THIRD_THREESHOLD, //MAX
    }

    public
    CachingDcMotorEx sMotor1, sMotor2;
    Sensors sensors;
    public PriorityMotor slideMotor;

    double[] manualPowers = new double[]{0.7, -0.7, 0.0};
    int indexManualPowers = 0;

    public static double kP = 0, kI = 0, kD = 0;
    public static double kgSlides = 0;

    double[] distancesThreeshold = new double[]{0, 0, 0}; //TODO: TUNE
    public static double lenght = 0;

    double vel = 0;
    double power = 0;

    public static double targetLength = 0;
    public static double ticksToInches = 0.0; //TUNE
    public static double maxSlidesHeight = 27.891; //TUNE
    final EricPid pid;

    public static double targetPosition = 0;
    public SlidesState slidesState = SlidesState.IDLE;

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue,Sensors sensors,Robot2 robot) {
        this.sensors = sensors;
        this.slidesState = SlidesState.IDLE;
        if (Globals.RUNMODE == Perioada.AUTO) {
            resetSlidesEncoder();
        }
        sMotor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "slides1"));
        sMotor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "slides2"));

        slideMotor = new PriorityMotor(
                new CachingDcMotorEx[]{sMotor1, sMotor2},
                "slides",
                3
        );

        hardwareQueue.addDevice(slideMotor);
        pid = new EricPid(kP, kI, kD);
        this.robot = robot;
    }

    void resetSlidesEncoder() {
        sMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sMotor1.setPower(0);
        sMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sMotor2.setPower(0);
        sMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;

        sMotor1.setPower(0);
        sMotor2.setPower(0);
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

    public void manual_extend() {
        slidesState = SlidesState.MANUAL;
        indexManualPowers = 0;
    }

    public void manual_retract() {
        slidesState = SlidesState.MANUAL;
        indexManualPowers = 1;
    }

    public void manual_IDLE() {
        indexManualPowers = 2;
        slidesState = SlidesState.IDLE;
    }

    public void AutoUpdate() {
        lenght = sensors.getSlidePos()* ticksToInches;
        vel = sensors.getSlideVelocity() * ticksToInches;
        if (abs(lenght - targetLength) < 0.5) {
            slidesState = SlidesState.IDLE;
        }
        power = pid.update(lenght) + kgSlides;
        slideMotor.setTargetPower(power);
    }

    public void checkForIntake() {
        if(robot.outtake.currentState == Outtake.FourBarState.TRANSFER_INTAKE) {
            firstThreeshold();
        }else if(robot.outtake.currentState == Outtake.FourBarState.OUTTAKE_POSITION) {
            secondThreeshold();
        }
    }
    public void update() {
        lenght = sensors.getSlidePos()* ticksToInches; //TODO:USE the sensor class for this
        vel = sensors.getSlideVelocity()     * ticksToInches;
        switch (slidesState) {
            case MANUAL:
                slideMotor.setTargetPower(manualPowers[indexManualPowers]);
                break;
            case IDLE:
                slideMotor.setTargetPower(0);
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
        slideMotor.setPowerForced(0);
    }

    public double getLength() {
        return lenght;
    }
}
