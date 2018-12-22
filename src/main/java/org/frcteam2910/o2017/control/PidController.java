package org.frcteam2910.o2017.control;

import org.frcteam2910.common.control.PidConstants;

@Deprecated
public class PidController {
    private PidConstants constants;

    private double setpoint;

    private boolean continuous = true;
    private double inputRange = 1.0;

    private double lastError = Double.NaN;

    public PidController(PidConstants constants) {
        this.constants = constants;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        if (continuous) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) {
                    error -= inputRange;
                } else {
                    error += inputRange;
                }
            }
        }

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
            lastError = error;
        }

        return constants.p * error + constants.d * derivative;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
}
