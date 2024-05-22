package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

public class ControllerUtils {

    public static void vibrateController(XboxController controller, double intensity, double durationSeconds) {
        controller.setRumble(RumbleType.kLeftRumble, intensity);
        controller.setRumble(RumbleType.kRightRumble, intensity);

        Timer timer = new Timer();
        timer.start();
        while (timer.get() < durationSeconds) {
            // Waiting for the duration to pass
        }

        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
    }
}
