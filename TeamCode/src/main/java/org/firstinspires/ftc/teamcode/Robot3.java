package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Globals.START_LOOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.BetaDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive.WolfPackDrive;
import org.firstinspires.ftc.teamcode.subsystems.EndGame.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo2;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake2;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake2;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides2;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

public class Robot3 {
    HardwareQueue hardwareQueue;

    public final MecanumDrive drive;
//    public final WolfPackDrive wolfPackDrive;

    public final Sensors sensors;

    public final Intake2 intake;

//    public final Plane plane;
    //public final BetaDrive drive;

    public final Slides2 slides;

    public final Extendo2 extendo;

    public final Outtake2 outtake;

    public Robot3(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();

        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        sensors = new Sensors(hardwareMap, hardwareQueue);

        intake = new Intake2(hardwareMap, hardwareQueue,this);

        outtake = new Outtake2(hardwareMap,hardwareQueue,this);

        extendo = new Extendo2(hardwareMap, hardwareQueue,sensors,this);

        slides = new Slides2(hardwareMap,hardwareQueue,sensors,this);

    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }
    public void updateSubsystems() {
        intake.Update.start();
        // plane.update();
        slides.slidesUpdate.start();
        extendo.update.start();
        outtake.outtakeUpdate.start();
        //drivetrain.updatePoseEstimate();
        sensors.update();

        //hardwareQueue.update();

    }


}
