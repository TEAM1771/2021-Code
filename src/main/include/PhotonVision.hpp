#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <iostream>
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

    photonlib::PhotonCamera camera { "gloworm" };
public:

    // Constructor
    PhotonCamera();

    enum class LED_Mode { Default   = -1,
                          Force_Off = 0,
                          Blink     = 2,
                          Force_On  = 1 };
    enum class Camera_Mode { Vision_Processor = 0,
                             Driver_Camera };
    enum class Stream_Mode { Standard = 0,
                             PiP_Main,
                             PiP_Secondary };
    enum class Snapshot_Mode { OFF = 0,
                               ON };

    void debug()
    {
        // std::cout << "DEBUG START" << table->GetNumber("FMSControlData", 0.0) << '\n';
        // auto tables = table->GetSubTables();
        // for (auto t : tables)
        //     std::cout << "DEBUG\t" << t << '\n';
    }

    [[nodiscard]] auto getLatestResult() const;

    [[nodiscard]] auto getTargetList() const;
    [[nodiscard]] auto getTargetList(photonlib::PhotonPipelineResult result) const;

    [[nodiscard]] auto getBestTarget() const;
    [[nodiscard]] auto getBestTarget(photonlib::PhotonPipelineResult result) const;

    [[nodiscard]] double getBestTargetX() const; // [-27.0, 27.0] For LimeLight-1
    [[nodiscard]] double getBestTargetX(photonlib::PhotonPipelineResult result) const;
    [[nodiscard]] double getTargetX(photonlib::PhotonTrackedTarget target) const;

    [[nodiscard]] double getBestTargetY() const; // [-20.5, 20.5] For LimeLight-1
    [[nodiscard]] double getBestTargetY(photonlib::PhotonPipelineResult result) const;
    [[nodiscard]] double getTargetY(photonlib::PhotonTrackedTarget target) const;

    [[nodiscard]] double getBestTargetArea() const; // % of image
    [[nodiscard]] double getBestTargetArea(photonlib::PhotonPipelineResult result) const;
    [[nodiscard]] double getTargetArea(photonlib::PhotonTrackedTarget target) const;

    [[nodiscard]] double getBestTargetSkew() const; // [-90, 0]
    [[nodiscard]] double getBestTargetSkew(photonlib::PhotonPipelineResult result) const;
    [[nodiscard]] double getTargetSkew(photonlib::PhotonTrackedTarget target) const;

    [[nodiscard]] auto getBestTargetPose() const; //not used for now but may be useful in the future
    [[nodiscard]] auto getBestTargetPose(photonlib::PhotonPipelineResult result) const;
    [[nodiscard]] auto getTargetPose(photonlib::PhotonTrackedTarget target) const;

    [[nodiscard]] bool hasTarget() const;


    void setLEDMode(LED_Mode mode);
    LED_Mode getLEDMode() const;
};
// inline PhotonCamera camera;
