package org.firstinspires.ftc.teamcode.subsystems.EndGame;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Caching.CachingServoImplEx;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;

@Config
public class Plane implements Subsystem {
    public enum status {
        IN,
        OUT
    }
    CachingServoImplEx plane;
    public status Status;
    double powerPosition = 0.69; //TODO:change this value

    public static double[] positions = new double[]{0.60,0.10};
    double close = 0.10;
    public Plane(HardwareMap hardwareMap){
        this.plane =
                new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "plane"));
        this.plane.setPosition(positions[0]);
        this.Status = status.IN;
    }

    public void hangMode() {
        this.plane.setPwmDisable();
    }

    public void setOut() {
        this.Status = status.OUT;
    }
    public void update() {
        switch (Status) {
            case IN:
                this.plane.setPosition(positions[0]);
                break;
            case OUT:
                this.plane.setPosition(positions[1]);
                break;

        }
    }

}
