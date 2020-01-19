package frc.robot.sensors;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor {

    private Rev2mDistanceSensor sensor;

    public DistanceSensor(Rev2mDistanceSensor.Port port) {
        sensor = new Rev2mDistanceSensor(port);
    }

    public void initialize() {
        sensor.setEnabled(true);
        sensor.setAutomaticMode(true);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
    }

    public double distance() {
        return sensor.getRange();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Distance", distance());
    }
}