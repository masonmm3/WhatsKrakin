package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;

public class IO {
public XboxController armController = new XboxController(1);

public IO() {}

public boolean getarmControllerAButton() {
    return armController.getAButton();
}

public boolean getarmControllerBButton() {
    return armController.getBButton();
}

public boolean getarmControllerXButton() {
    return armController.getXButton();
}

public boolean getarmControllerYButton() {
    return armController.getYButton();
}

}