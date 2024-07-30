package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

public abstract class AutoBase extends LinearOpMode {
    protected Robot2 robot;
    protected AutoActionScheduler sched;
    /*protected int w = 640;
    protected int h = 480;*/

    public static int spike = -1;

    final public void update() {
        int pixelCount = robot.sensors.pixelCounter();
        telemetry.addData("Time left", 30 - getRuntime());
        telemetry.addData("Pixel Count", pixelCount);
        robot.update();
        telemetry.update();
    }


    @Override
    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing");
        telemetry.update();


        Memory.LAST_POSE = getStartPose();
        Memory.RAN_AUTO = true;
        robot.drivetrain.pose = Memory.LAST_POSE;

        sched = new AutoActionScheduler(this::update);
        resetRuntime();
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME);

        if (isStopRequested()) return;

        robot.drivetrain.updatePoseEstimate();
        robot.drivetrain.pose = getStartPose();
        Memory.FINISHED_AUTO = false;
        onRun();
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            Memory.FINISHED_AUTO = true;
            return false;
        }));
        robot.drivetrain.pose = getStartPose();
        //drive.imu.resetYaw();

        sched.run();
        Memory.LAST_POSE = robot.drivetrain.pose;

        Log.d("Auto", "Auto ended at " + getRuntime());
    }
    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();
    protected void onInit() {}
    protected abstract void onRun();
}
