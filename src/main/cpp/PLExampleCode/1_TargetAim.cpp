/**
 * Copyright (C) 2018-2020 Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// This program has a ton of errors that I can't be bothered with fixing atm

/*                                                       ******** Commented everything out to get rid of errors ********
#include "Robot.hpp"
#include "PhotonLib/PhotonPipelineResult.hpp"
#include <PhotonLib/PhotonUtils.hpp>

void Robot::TeleopPeriodic() {
  double forwardSpeed = // Controller (what)
      -1.0 * xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand); // Dualshock?
  double rotationSpeed;

  if (xboxController.GetAButton()) {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();

    if (result.HasTargets()) {
      // Rotation speed is the output of the PID controller
      rotationSpeed = -1.0 * controller.Calculate(result.GetBestTarget().GetYaw(), 0); // More controller stuff
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
  } else {
    // Manual Driver Mode
    rotationSpeed =
        xboxController.GetX(frc::GenericHID::JoystickHand::kLeftHand);
  }

  // Use our forward/turn speeds to control the drivetrain
  drive.ArcadeDrive(forwardSpeed, rotationSpeed);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
*/