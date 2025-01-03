package org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .002968434003;
        ThreeWheelConstants.strafeTicksToInches = .002968434003;
        ThreeWheelConstants.turnTicksToInches = .002968434003;
        ThreeWheelConstants.leftY = 4.8228346;
        ThreeWheelConstants.rightY = -4.8228346;
        ThreeWheelConstants.strafeX = -6.0279527559;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "lfd";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rfd";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "lbd";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




