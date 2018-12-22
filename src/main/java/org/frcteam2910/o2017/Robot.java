package org.frcteam2910.o2017;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;
import org.frcteam2910.o2017.subsystems.Drivetrain;

public class Robot extends IterativeRobot {
    public static final double KINEMATICS_PERIOD = 5 * 1e-3; // 5 ms

	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final SubsystemManager subsystemManager = new SubsystemManager(drivetrain);

	private final OI oi = OI.getInstance();

	private final Logger logger = new Logger(Robot.class);

	@Override
	public void robotInit() {
		try {
			logger.info("Initializing robot code");

			oi.bindControls();
		} catch (Throwable t) {
			new Logger("robotInit").error(t);
			throw t;
		}
	}

	@Override
	public void autonomousInit() {
		try {
			logger.info("Beginning autonomous mode");

			subsystemManager.zeroSensors();
			subsystemManager.enableKinematicLoop(KINEMATICS_PERIOD);
		} catch (Throwable t) {
			new Logger("autonomousInit").error(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}

	@Override
	public void disabledInit() {
		try {
			logger.info("Disabling robot");

			subsystemManager.disableKinematicLoop();
			subsystemManager.stop();

			allPeriodic();
		} catch (Throwable t) {
			new Logger("disabledInit").error(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		allPeriodic();
	}

	@Override
	public void teleopInit() {
		try {
			logger.info("Beginning teleoperated mode");

			subsystemManager.zeroSensors();
			subsystemManager.enableKinematicLoop(KINEMATICS_PERIOD);

			drivetrain.holonomicDrive(Vector2.ZERO, 0);
		} catch (Throwable t) {
			new Logger("teleopInit").error(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			Scheduler.getInstance().run();
			allPeriodic();
		} catch (Throwable t) {
			new Logger("teleopPeriodic").error(t);
			throw t;
		}
	}

	public void allPeriodic() {
		subsystemManager.outputToSmartDashboard();
		subsystemManager.writeToLog();

		Scheduler.getInstance().run();
	}
}
