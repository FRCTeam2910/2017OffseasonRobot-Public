package org.frcteam2910.o2017.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Constants;
import org.frcteam2910.o2017.control.PidController;

public class OnboardPidSwerveModule extends SwerveModule {
    private final TalonSRX angleMotor;
    private final TalonSRX driveMotor;

    private final PidController angleController = new PidController(new PidConstants(5.0, 0.0, 50.0));

    private volatile double angleReading = 0.0;
    private volatile double driveReading = 0.0;

    private volatile double angleOutput = 0.0;
    private volatile double driveOutput = 0.0;

    private double lastSensorUpdateTime = Timer.getFPGATimestamp();
    private final Notifier canThread;

    public OnboardPidSwerveModule(Vector2 modulePosition, double adjustmentAngle,
                                  TalonSRX angleMotor, TalonSRX driveMotor) {
        super(modulePosition, Rotation2.fromDegrees(adjustmentAngle));
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;

        angleMotor.setInverted(true);
        angleMotor.setSensorPhase(true);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.CAN_TIMEOUT_MS);

        canThread = new Notifier(() -> {
                double now = Timer.getFPGATimestamp();
                double dt = now - lastSensorUpdateTime;
                lastSensorUpdateTime = now;
                double rate = 1.0 / dt;

                SmartDashboard.putNumber(String.format("%s module update rate", getName()), rate);

                double newAngle = angleMotor.getSelectedSensorPosition(0) / 1024.0;
                double newDrive = driveMotor.getSelectedSensorPosition(0);

                angleReading = newAngle;
                driveReading = newDrive;

                angleMotor.set(ControlMode.PercentOutput, angleOutput);
                driveMotor.set(ControlMode.PercentOutput, driveOutput);
        });
        canThread.startPeriodic(5e-3);
    }

    @Override
    protected double getAngleEncoderRotations() {
//        return angleEncoderSupplier.getAsDouble();
//        synchronized (sensorLock) {
//            return angleReading;
//        }
        return angleReading;
    }

    @Override
    protected void setTargetAngleRotations(double rotations) {
        double modRot = rotations % 1.0;
        if (modRot < 0.0) {
            modRot += 1.0;
        }
        SmartDashboard.putNumber(String.format("%s module target angle", getName()), modRot * 2 * Math.PI);

        synchronized (angleController) {
            angleController.setSetpoint(rotations);
        }
    }

    @Override
    protected void setDriveMotorInverted(boolean inverted) {
        driveMotor.setInverted(inverted);
    }

    @Override
    public double getCurrentDrivePercentage() {
        return driveOutput;
    }

    @Override
    public void setTargetDrivePercentage(double percentage) {
//        driveMotor.set(percentage);
        driveOutput = percentage;
    }

    @Override
    public void zeroDistance() {

    }

    @Override
    public double getCurrentDistance() {
//        return driveEncoderSupplier.getAsDouble();
//        synchronized (sensorLock) {
//            return driveReading;
//        }
        return driveReading;
    }

    @Override
    public double getCurrentRate() {
        return 0;
    }

    @Override
    public void update(double dt) {
        double currentAngle = getAngleEncoderRotations();
        double modAngle = currentAngle % 1.0;
        if (modAngle < 0) {
            modAngle += 1.0;
        }

        SmartDashboard.putNumber(String.format("%s module angle", getName()), modAngle * 2 * Math.PI);

        synchronized (angleController) {
            angleOutput = angleController.calculate(getAngleEncoderRotations(), dt);
        }

        SmartDashboard.putNumber(String.format("%s module angle output", getName()), angleOutput);
//        angleMotor.set(angleOutput * 1e-3);
    }
}
