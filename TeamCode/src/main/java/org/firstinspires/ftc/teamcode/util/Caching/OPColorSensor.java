package org.firstinspires.ftc.teamcode.util.Caching;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OPColorSensor {
    public ColorRangeSensor delegate;
    private double distance = 0;
    private double alpha = 0;

    public OPColorSensor(ColorRangeSensor sensor) {
        delegate = sensor;
    }

    public double getDistance() {
        return distance;
    }

    public double alpha() {
        return alpha;
    }

    public void update() {
        distance = delegate.getDistance(DistanceUnit.MM);
    }

    public void close() {
        delegate.close();
    }
}