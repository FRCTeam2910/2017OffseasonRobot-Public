package org.frcteam2910.o2017.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SPI;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.commands.HolonomicDriveCommand;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.OnboardPid2017SwerveModule;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.o2017.OI;

import static org.frcteam2910.o2017.RobotMap.*;

public class Drivetrain extends SwerveDrivetrain {
	private static final Drivetrain instance = new Drivetrain();

	private final SwerveModule frontLeftModule = new OnboardPid2017SwerveModule(new Vector2(-TRACKWIDTH / 2, WHEELBASE / 2),
			DRIVETRAIN_FRONT_LEFT_OFFSET,
			new WPI_TalonSRX(DRIVETRAIN_FRONT_LEFT_ANGLE),
			new WPI_TalonSRX(DRIVETRAIN_FRONT_LEFT_DRIVE));
	private final SwerveModule frontRightModule = new OnboardPid2017SwerveModule(new Vector2(TRACKWIDTH / 2, WHEELBASE / 2),
			DRIVETRAIN_FRONT_RIGHT_OFFSET,
			new WPI_TalonSRX(DRIVETRAIN_FRONT_RIGHT_ANGLE),
			new WPI_TalonSRX(DRIVETRAIN_FRONT_RIGHT_DRIVE));
	private final SwerveModule backLeftModule = new OnboardPid2017SwerveModule(new Vector2(-TRACKWIDTH / 2, -WHEELBASE / 2),
			DRIVETRAIN_BACK_LEFT_OFFSET,
			new WPI_TalonSRX(DRIVETRAIN_BACK_LEFT_ANGLE),
			new WPI_TalonSRX(DRIVETRAIN_BACK_LEFT_DRIVE));
	private final SwerveModule backRightModule = new OnboardPid2017SwerveModule(new Vector2(TRACKWIDTH / 2, -WHEELBASE / 2),
			DRIVETRAIN_BACK_RIGHT_OFFSET,
			new WPI_TalonSRX(DRIVETRAIN_BACK_RIGHT_ANGLE),
			new WPI_TalonSRX(DRIVETRAIN_BACK_RIGHT_DRIVE));
	private final SwerveModule[] swerveModules = {
		frontLeftModule, frontRightModule,
		backLeftModule, backRightModule
	};

	private final NavX navX = new NavX(SPI.Port.kMXP);

	private Drivetrain() {
		// NavX is mounted upside-down to make the gyro angle
		// increase when the robot is rotated counter-clockwise
		navX.setInverted(true);

		frontLeftModule.setName("Front Left");
		frontRightModule.setName("Front Right");
		backLeftModule.setName("Back Left");
		backRightModule.setName("Back Right");
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
