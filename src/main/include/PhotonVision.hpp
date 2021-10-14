#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

// include all the PhotonLib HPP files
#include "PhotonLib/Packet.hpp"
#include "PhotonLib/PhotonCamera.hpp"
#include "PhotonLib/PhotonPipelineResult.hpp"
#include "PhotonLib/PhotonTrackedTarget.hpp"
#include "PhotonLib/PhotonUtils.hpp"
#include "PhotonLib/SimPhotonCamera.hpp"
#include "PhotonLib/SimVisionSystem.hpp"
#include "PhotonLib/SimVisionTarget.hpp"

class PhotonCamera
{
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("photonvision");

public:
    enum class LED_Mode { Keep_Current = -1,
                          Force_Off=0,
                          Blink=2,
                          Force_On =1 };
    enum class Camera_Mode { Vision_Processor = 0,
                             Driver_Camera };
    enum class Stream_Mode { Standard = 0,
                             PiP_Main,
                             PiP_Secondary };
    enum class Snapshot_Mode { OFF = 0,
                               ON };

    [[nodiscard]] double getX() const;    // [-27.0, 27.0] For LimeLight-1
    [[nodiscard]] double getY() const;    // [-20.5, 20.5] For LimeLight-1
    [[nodiscard]] double getArea() const; // % of image
    [[nodiscard]] double getSkew() const; // [-90, 0]
    [[nodiscard]] bool   hasTarget() const;

    void setLEDMode(LED_Mode mode);
};