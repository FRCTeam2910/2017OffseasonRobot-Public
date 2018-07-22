package org.frcteam2910.o2017;

import edu.wpi.first.wpilibj.buttons.Button;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.o2017.subsystems.Drivetrain;

public class OI {
	private static final OI instance = new OI();

	private final Controller controller = new XboxController(0);

	private OI() { }

	public static OI getInstance() {
		return instance;
	}

	public void bindControls() {
		getCalibrateFieldOrientedButton().whenPressed(new ZeroFieldOrientedCommand(Drivetrain.getInstance()));

		getDrivetrainRotationAxis().setInverted(true);
		getDrivetrainStrafeAxis().setInverted(true);

		getDrivetrainForwardAxis().setScale(0.5);
		getDrivetrainStrafeAxis().setScale(0.5);
		getDrivetrainRotationAxis().setScale(0.05);
	}

	public Axis getDrivetrainForwardAxis() {
		return controller.getLeftYAxis();
	}

	public Axis getDrivetrainStrafeAxis() {
		return controller.getLeftXAxis();
	}

	public Axis getDrivetrainRotationAxis() {
		return controller.getRightXAxis();
	}

	public Button getFieldOrientedOverrideButton() {
		return controller.getLeftBumperButton();
	}

	public Button getCalibrateFieldOrientedButton() {
		return controller.getStartButton();
	}
}
