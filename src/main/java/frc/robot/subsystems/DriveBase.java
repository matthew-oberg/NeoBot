package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.sensors.Limelight;
import frc.robot.sensors.Pipeline;

public class DriveBase {

    private CANSparkMax leftOne, leftTwo, rightOne, rightTwo;

    private AHRS gyro = new AHRS();

    private final double pTurn = 0.005;
    private final double iTurn = 0;
    private final double dTurn = 0;

    private final double pDrive = 0.016;
    private final double iDrive = 0;
    private final double dDrive = 0;

    private PIDController turnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController driveController = new PIDController(pDrive, iDrive, dDrive);
    private PIDController ballTurnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController ballDriveController = new PIDController(pDrive, iDrive, dDrive);

    private Limelight camera = new Limelight(Pipeline.BALL);

    private double tx_prev = 0;

    public DriveBase(int leftOnePort, int leftTwoPort, int rightOnePort, int rightTwoPort) {
        this.leftOne = new CANSparkMax(leftOnePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.leftTwo = new CANSparkMax(leftTwoPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightOne = new CANSparkMax(rightOnePort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightTwo = new CANSparkMax(rightTwoPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void initialize() {
        leftOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftOne.setOpenLoopRampRate(0.5);
        leftTwo.setOpenLoopRampRate(0.5);
        rightOne.setOpenLoopRampRate(0.5);
        rightTwo.setOpenLoopRampRate(0.5);

        leftOne.getEncoder().setPositionConversionFactor(34.19);
        leftTwo.getEncoder().setPositionConversionFactor(34.19);
        rightOne.getEncoder().setPositionConversionFactor(34.19);
        rightTwo.getEncoder().setPositionConversionFactor(34.19);

        turnController.setSetpoint(0);
        turnController.setTolerance(2);

        driveController.setSetpoint(0);
        driveController.setTolerance(5);

        ballTurnController.setSetpoint(0);
        ballTurnController.setTolerance(0.25);

        ballDriveController.setSetpoint(10);
        ballDriveController.setTolerance(0.5);

        reset();
    }

    public void reset() {
        leftOne.getEncoder().setPosition(0);
        leftTwo.getEncoder().setPosition(0);
        rightOne.getEncoder().setPosition(0);
        rightTwo.getEncoder().setPosition(0);

        gyro.reset();

        turnController.setSetpoint(0);
        driveController.setSetpoint(0);

        tx_prev = 0;
    }

    public void arcadeDrive(XboxController controller, GenericHID.Hand left, GenericHID.Hand right) {
        double throttle = 0;
        double turn = 0;

        if (controller.getY(left) > 0.05 || controller.getY(left) < -0.05) {
            throttle = controller.getY(left);
        }

        if (controller.getX(right) > 0.05 || controller.getX(right) < -0.05) {
            turn = controller.getX(right);
        }

        leftOne.set(turn - throttle);
        leftTwo.set(turn - throttle);

        rightOne.set(turn + throttle);
        rightTwo.set(turn + throttle);
    }

    public void angleLockedDrive(XboxController controller, GenericHID.Hand left) {
        double throttle = 0;
        double turn = turnController.calculate(getAngle());

        if (controller.getY(left) > 0.05 || controller.getY(left) < -0.05) {
            throttle = controller.getY(left);
        }

        leftOne.set(turn - throttle);
        leftTwo.set(turn - throttle);

        rightOne.set(turn + throttle);
        rightTwo.set(turn + throttle);
    }

    public void snapToAngle() {
        double turn = turnController.calculate(getAngle());

        leftOne.set(turn);
        leftTwo.set(turn);

        rightOne.set(turn);
        rightTwo.set(turn);
    }

    public void ballFollowDrive() {
        if (camera.getValues()[1] != 0) {
            tx_prev = camera.getValues()[1];
        }

        if (camera.getValues()[0] == 0) {
            ballSeekDrive();
        } else {
            double throttle = -ballDriveController.calculate(camera.getValues()[2]);
            double turn = -ballTurnController.calculate(camera.getValues()[1]);

            leftOne.set(turn - throttle);
            leftTwo.set(turn - throttle);

            rightOne.set(turn + throttle);
            rightTwo.set(turn + throttle);
        }
    }

    private void ballSeekDrive() {
        if (tx_prev > 0) {
            leftOne.set(0.2);
            leftTwo.set(0.2);

            rightOne.set(0.2);
            rightTwo.set(0.2);
        } else if (tx_prev < 0) {
            leftOne.set(-0.2);
            leftTwo.set(-0.2);

            rightOne.set(-0.2);
            rightTwo.set(-0.2);
        }
    }

    public void distanceDrive() {
        double throttle = driveController.calculate(getLeftPosition());

        leftOne.set(throttle);
        leftTwo.set(throttle);

        rightOne.set(-throttle);
        rightTwo.set(-throttle);
    }

    public void setAngle(double angle) {
        turnController.setSetpoint(angle);
    }

    public void setDistance(double degrees) {
        driveController.setSetpoint(degrees);
    }

    public double getLeftPosition() {
        return leftOne.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return rightOne.getEncoder().getPosition();
    }

    public double getLeftVelocity() {
        return leftOne.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightOne.getEncoder().getVelocity();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public boolean turnOnTarget() {
        return turnController.atSetpoint();
    }

    public boolean driveOnTarget() {
        return driveController.atSetpoint();
    }

    public boolean ballTurnOnTarget() {
        return ballTurnController.atSetpoint();
    }

    public boolean ballDriveOnTarget() {
        return ballDriveController.atSetpoint();
    }

    public void dashboard() {
        SmartDashboard.putData("Turn Controller", turnController);
        SmartDashboard.putData("Drive Controller", driveController);
        SmartDashboard.putData("Ball Turn Controller", ballTurnController);
        SmartDashboard.putData("Ball Drive Controller", ballDriveController);

        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());

        SmartDashboard.putNumber("Angle", getAngle());

        SmartDashboard.putNumber("tv", camera.getValues()[0]);
        SmartDashboard.putNumber("tx", camera.getValues()[1]);
        SmartDashboard.putNumber("ta", camera.getValues()[2]);
    }
}