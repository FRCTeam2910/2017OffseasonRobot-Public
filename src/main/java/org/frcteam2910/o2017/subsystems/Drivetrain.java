package org.frcteam2910.o2017.subsystems;

import edu.wpi.first.wpilibj.SPI;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.commands.HolonomicDriveCommand;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.SwerveModule2017;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.o2017.OI;

import static org.frcteam2910.o2017.RobotMap.*;

public class Drivetrain extends SwerveDrivetrain {
	private static final Drivetrain instance = new Drivetrain();

	private final SwerveModule[] swerveModules = new SwerveModule[]{
			new SwerveModule2017(new Vector2(-TRACKWIDTH / 2, WHEELBASE / 2),
					DRIVETRAIN_FRONT_LEFT_OFFSET,
					DRIVETRAIN_FRONT_LEFT_ANGLE,
					DRIVETRAIN_FRONT_LEFT_DRIVE),
			new SwerveModule2017(new Vector2(TRACKWIDTH / 2, WHEELBASE / 2),
					DRIVETRAIN_FRONT_RIGHT_OFFSET,
					DRIVETRAIN_FRONT_RIGHT_ANGLE,
					DRIVETRAIN_FRONT_RIGHT_DRIVE),
			new SwerveModule2017(new Vector2(-TRACKWIDTH / 2, -WHEELBASE / 2),
					DRIVETRAIN_BACK_LEFT_OFFSET,
					DRIVETRAIN_BACK_LEFT_ANGLE,
					DRIVETRAIN_BACK_LEFT_DRIVE),
			new SwerveModule2017(new Vector2(TRACKWIDTH / 2, -WHEELBASE / 2),
					DRIVETRAIN_BACK_RIGHT_OFFSET,
					DRIVETRAIN_BACK_RIGHT_ANGLE,
					DRIVETRAIN_BACK_RIGHT_DRIVE)
	};

	private final NavX navX = new NavX(SPI.Port.kMXP);

	private Drivetrain() {
		// NavX is mounted upside-down to make the gyro angle
		// increase when the robot is rotated counter-clockwise
		navX.setInverted(true);

		swerveModules[0].setInverted(true);
		swerveModules[2].setInverted(true);

		swerveModules[0].setName("Front Left");
		swerveModules[1].setName("Front Right");
		swerveModules[2].setName("Back Left");
		swerveModules[3].setName("Back Right");
	}

	public static Drivetrain getInstance() {
		return instance;
	}

	@Override
	public SwerveModule[] getSwerveModules() {
		return swerveModules;
	}

	@Override
	public Gyroscope getGyroscope() {
		return navX;
	}

	@Override
	public double getMaximumVelocity() {
		return 12;
	}

	@Override
	public double getMaximumAcceleration() {
		return 5;
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new HolonomicDriveCommand(this,
				OI.getInstance().getDrivetrainForwardAxis(),
				OI.getInstance().getDrivetrainStrafeAxis(),
				OI.getInstance().getDrivetrainRotationAxis(),
				OI.getInstance().getFieldOrientedOverrideButton()));
	}
}
