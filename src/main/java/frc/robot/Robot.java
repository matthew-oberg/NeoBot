package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.DistanceSensor;
import frc.robot.subsystems.DriveBase;

public class Robot extends TimedRobot {

    public static XboxController controller = new XboxController(0);
    public static final GenericHID.Hand left = GenericHID.Hand.kLeft;
    public static final GenericHID.Hand right = GenericHID.Hand.kRight;

    public static DriveBase base = new DriveBase(3, 4, 1, 2);

    //public static ColorSensor color = new ColorSensor(I2C.Port.kOnboard);

    public static DistanceSensor distance = new DistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    @Override
    public void robotInit() {
        base.initialize();
    }

    @Override
    public void teleopInit() {
        base.initialize();
        base.reset();
        distance.initialize();
    }

    @Override
    public void teleopPeriodic() {
        reseter();

        setAngle();
        
        drive();

        dashboard();
    }

    public void drive() {
        if (controller.getTriggerAxis(right) > 0.5) {
            base.angleLockedDrive(controller, left);
        } else if (controller.getTriggerAxis(left) > 0.5) {
            base.ballFollowDrive();
        } else if (controller.getBumper(left)) {
            base.distanceDrive();
        } else {
            base.arcadeDrive(controller, left, right);
        }
    }

    public void dashboard() {
        base.dashboard();
        //color.dashboard();
        distance.dashboard();
    }

    public void setAngle() {
        if (controller.getPOV() == 0) {
            base.setAngle(0);
        } else if (controller.getPOV() == 90) {
            base.setAngle(90);
        } else if (controller.getPOV() == 180) {
            base.setAngle(180);
        } else if (controller.getPOV() == 270) {
            base.setAngle(270);
        }
    }

    public void reseter() {
        if (controller.getAButtonReleased()) {
            base.reset();
        }
    }
}