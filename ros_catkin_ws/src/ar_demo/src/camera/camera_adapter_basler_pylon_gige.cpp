//! Unified camera node - Basler Pylon GigE adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_adapter_basler_pylon_gige.hpp"


namespace ar_demo {

  BaslerPylonGigECameraAdapter::BaslerPylonGigECameraAdapter(std::shared_ptr<const Configuration> configuration):
		CameraAdapter{configuration},
    camera_is_initialised_{false},
    camera_name_{""},
    camera_identification_{""},
    camera_is_streaming_{false},
    pait_{},
    camera_{nullptr},
    format_converter_{} {
    ROS_INFO("Using Basler Pylon GigE camera adapter.");
  }

  BaslerPylonGigECameraAdapter::~BaslerPylonGigECameraAdapter() {
    if(camera_) {
      camera_->StopGrabbing();
      camera_->Close();
      camera_->DestroyDevice();
      delete camera_;
    }
  }

  bool BaslerPylonGigECameraAdapter::initialise(const std::string & camera_name, const std::string & model,
      const std::string & serial_number, const std::string & user_defined_name, ImageMode image_mode) {
    camera_is_initialised_ = false;
    camera_name_ = "";
    try {
      // Initialise Basler device filter.
      Pylon::CDeviceInfo deviceInfoFilter;
      if(model != "") {
        deviceInfoFilter.SetModelName(model.c_str());
      }
      if(serial_number != "") {
        deviceInfoFilter.SetSerialNumber(serial_number.c_str());
      }
      if(user_defined_name != "") {
        deviceInfoFilter.SetUserDefinedName(user_defined_name.c_str());
      }
      Pylon::DeviceInfoList_t deviceInfoFilterList;
      deviceInfoFilterList.push_back(deviceInfoFilter);
      // Get all devices which pass our filter.
      Pylon::DeviceInfoList_t deviceInfoList;
      int deviceCount = Pylon::CTlFactory::GetInstance().EnumerateDevices(deviceInfoList, deviceInfoFilterList);
      // Check that we found exactly one camera.
      if (deviceCount == 0) {
        ROS_FATAL("No matching camera found.");
        throw std::runtime_error("No matching camera found");
      } else if (deviceCount > 1) {
        ROS_FATAL("Multiple matching cameras found.");
        throw std::runtime_error("Multiple matching cameras found");
      }
      // Show the identification of the camera we found.
      ROS_INFO("Using camera with model name '%s', serial number '%s' and "
          "user defined name '%s'.", std::string(deviceInfoList[0].GetModelName()).c_str(),
          std::string(deviceInfoList[0].GetSerialNumber()).c_str(),
          std::string(deviceInfoList[0].GetUserDefinedName()).c_str());
      camera_identification_ = std::string(deviceInfoList[0].GetModelName()) + "_"
        + std::string(deviceInfoList[0].GetSerialNumber());
      // Create instant camera object.
      camera_ = new Pylon::CBaslerGigEInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(deviceInfoList[0]));
      // Setup camera.
      camera_->Open();
      // Set heartbeat timeout to 1s (if the camera supports it).
      GenApi::CIntegerPtr heartbeat_timeout{camera_->GetTLNodeMap().GetNode("HeartbeatTimeout")};
      if(!heartbeat_timeout.IsValid()) {
        int64_t raw_value = 1000;
        int64_t corrected_value = raw_value - (raw_value % heartbeat_timeout->GetInc());
        heartbeat_timeout->SetValue(corrected_value);
        ROS_INFO("Set heartbeat timeout to %ldms.", corrected_value);
      }
      else {
        ROS_INFO("Heartbeat timeout not supported by camera.");
      }
      // Initialise image format converter.
      if(image_mode == ImageMode::BGR888) {
        format_converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;
      }
      else if(image_mode == ImageMode::GRAY8) {
        format_converter_.OutputPixelFormat = Pylon::PixelType_Mono8;
        format_converter_.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
      }
      // Get additional information for camera.
      image_transmission_time_ = ros::Duration(configuration_->getCameraImageTransmissionTime(camera_name).second);
      aperture_ = configuration_->getCameraAperture(camera_name).second;
      ev_correction_ = configuration_->getCameraExposureValueCorrection(camera_name).second;
      // Camera is initialised correctly.
      camera_name_ = camera_name;
      camera_is_initialised_ = true;
      return true;
    }
    catch(std::exception & exception) {
      ROS_FATAL("An error occured during the initialisation of the camera.");
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isInitialised() {
    return camera_is_initialised_;
  }

  std::string BaslerPylonGigECameraAdapter::getCameraIdentification() {
    if(camera_is_initialised_) {
      return camera_identification_;
    }
    else {
      return "";
    }
  }

  bool BaslerPylonGigECameraAdapter::setManualExposure(double time_s, double gain) {
    if(camera_is_initialised_) {
      // Exposure time.
      camera_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
      camera_->ExposureTimeAbs.SetValue(time_s * 1.e6);
      ROS_INFO("Exposure time: %.2f µs.", camera_->ExposureTimeAbs.GetValue());
      // Analog gain.
      camera_->GainAuto.SetValue(Basler_GigECameraParams::GainAuto_Off);
      camera_->GainSelector.SetValue(Basler_GigECameraParams::GainSelector_All);
      camera_->GainRaw.SetValue(int64_t(gain)); // TODO: check camera specific vs dB.
      ROS_INFO("Gain: %ld (unit unknown).", camera_->GainRaw.GetValue());
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isAutoExposureSupported() {
    return true;
  }

  bool BaslerPylonGigECameraAdapter::setAutoExposure(double target_brightness, double time_s_min, double time_s_max,
      double gain_db_min, double gain_db_max) {
    if(camera_is_initialised_) {
      // Set target brightness.
      try {
        camera_->AutoTargetValue.SetValue(int(target_brightness * 255.));
        target_brightness = camera_->AutoTargetValue.GetValue() / 255.;
        ROS_INFO("Target brightness: %.4f.", target_brightness);
      }
      catch(...) {
        ROS_WARN("Failed to set requested target brightness %.4f. Current target brightness: %.4f.",
            target_brightness, camera_->AutoTargetValue.GetValue() / 255.);
      }
      // Set exposure time
      if(time_s_min < camera_->AutoExposureTimeAbsUpperLimit.GetValue() / 1.e6) {
        // If the new lower limit is smaller than the old upper limit, set the lower limit first.
        camera_->AutoExposureTimeAbsLowerLimit.SetValue(time_s_min * 1.e6);
        camera_->AutoExposureTimeAbsUpperLimit.SetValue(time_s_max * 1.e6);
      }
      else {
        // If the new lower limit is larger then the old upper limit, set the upper limit first.
        camera_->AutoExposureTimeAbsUpperLimit.SetValue(time_s_max * 1.e6);
        camera_->AutoExposureTimeAbsLowerLimit.SetValue(time_s_min * 1.e6);
      }
      camera_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Continuous);
      ROS_INFO("Exposure time: %.3f s ... %.3f s.", time_s_min, time_s_max);
      // Set gain.
      if(int64_t(gain_db_min) < camera_->AutoGainRawUpperLimit.GetValue()) {
        camera_->AutoGainRawLowerLimit.SetValue(int64_t(gain_db_min));
        camera_->AutoGainRawUpperLimit.SetValue(int64_t(gain_db_max));
      }
      else {
        camera_->AutoGainRawUpperLimit.SetValue(int64_t(gain_db_max));
        camera_->AutoGainRawLowerLimit.SetValue(int64_t(gain_db_min));
      }
      camera_->GainAuto.SetValue(Basler_GigECameraParams::GainAuto_Continuous);
      ROS_INFO("Analog gain: %ld ... %ld (unit unknown).", int64_t(gain_db_min), int64_t(gain_db_max));
      // Set damping.
      camera_->GrayValueAdjustmentDampingRaw.SetValue(700); // Default value is 700.
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isAutoExposureRoiSupported() {
    return true;
  }

  bool BaslerPylonGigECameraAdapter::setAutoExposureRoi(int column, int row, int width, int height) {
    if(camera_is_initialised_) {
      if(column == -1 && row == -1 && width == -1 && height == -1) {
        column = camera_->OffsetX.GetValue();
        row = camera_->OffsetY.GetValue();
        width = camera_->Width.GetValue();
        height = camera_->Height.GetValue();
      }
      // Enable first ROI for Intensity
      camera_->AutoFunctionAOISelector.SetValue(Basler_GigECameraParams::AutoFunctionAOISelector_AOI1);
      camera_->AutoFunctionAOIWidth.SetValue(1);
      camera_->AutoFunctionAOIHeight.SetValue(1);
      camera_->AutoFunctionAOIOffsetX.SetValue(0);
      camera_->AutoFunctionAOIOffsetY.SetValue(0);
      camera_->AutoFunctionAOIWidth.SetValue(width);
      camera_->AutoFunctionAOIHeight.SetValue(height);
      camera_->AutoFunctionAOIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionAOIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto brightness.
      camera_->AutoFunctionAOIUsageIntensity.SetValue(true);
      camera_->AutoFunctionAOIUsageWhiteBalance.SetValue(false);
      ROS_INFO("Auto exposure ROI: (%d, %d) - (%d, %d).", column, row, column + width - 1, row + height -1);
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::setManualWhiteBalance(double red, double green, double blue) {
    if(camera_is_initialised_) {
      camera_->ColorTransformationSelector.SetValue(Basler_GigECameraParams::ColorTransformationSelector_RGBtoRGB);
      camera_->BalanceWhiteAuto.SetValue(Basler_GigECameraParams::BalanceWhiteAuto_Off);
      // camera_->LightSourceSelector.SetValue(Basler_GigECameraParams::LightSourceSelector_Tungsten2800K);
      // camera_->LightSourceSelector.SetValue(Basler_GigECameraParams::LightSourceSelector_Daylight5000K);
      camera_->LightSourceSelector.SetValue(Basler_GigECameraParams::LightSourceSelector_Off);
      camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Red);
      camera_->BalanceRatioAbs.SetValue(red);
      camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Green);
      camera_->BalanceRatioAbs.SetValue(green);
      camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Blue);
      camera_->BalanceRatioAbs.SetValue(blue);
      ROS_INFO("White balance: R: %.2f, G: %.2f, B: %.2f.", red, green, blue);
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isAutoWhiteBalanceSupported() {
    return true;
  }

  bool BaslerPylonGigECameraAdapter::setAutoWhiteBalance() {
    if(camera_is_initialised_) {
      // AWB should be off to change its settings (otherwise, Pylon throws an exception).
      camera_->BalanceWhiteAuto.SetValue(Basler_GigECameraParams::BalanceWhiteAuto_Off);
      camera_->LightSourceSelector.SetValue(Basler_GigECameraParams::LightSourceSelector_Off);
      camera_->ColorTransformationSelector.SetValue(Basler_GigECameraParams::ColorTransformationSelector_RGBtoRGB);
      camera_->BalanceWhiteAuto.SetValue(Basler_GigECameraParams::BalanceWhiteAuto_Continuous);
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isAutoWhiteBalanceRoiSupported() {
    return true;
  }

  bool BaslerPylonGigECameraAdapter::setAutoWhiteBalanceRoi(int column, int row, int width, int height) {
    if(camera_is_initialised_) {
      if(column == -1 && row == -1 && width == -1 && height == -1) {
        column = camera_->OffsetX.GetValue();
        row = camera_->OffsetY.GetValue();
        width = camera_->Width.GetValue();
        height = camera_->Height.GetValue();
      }
      // Enable second ROI for Intensity
      camera_->AutoFunctionAOISelector.SetValue(Basler_GigECameraParams::AutoFunctionAOISelector_AOI2);
      camera_->AutoFunctionAOIOffsetX.SetValue(0);
      camera_->AutoFunctionAOIOffsetY.SetValue(0);
      camera_->AutoFunctionAOIWidth.SetValue(width);
      camera_->AutoFunctionAOIHeight.SetValue(height);
      camera_->AutoFunctionAOIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionAOIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto brightness.
      camera_->AutoFunctionAOIUsageWhiteBalance.SetValue(true);
      camera_->AutoFunctionAOIUsageIntensity.SetValue(false);
      ROS_INFO("Auto white balance ROI: (%d, %d) - (%d, %d).", column, row, column + width - 1, row + height -1);
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::setFrameRate(double frame_rate_hz) {
    if(camera_is_initialised_) {
      try {
        camera_->AcquisitionFrameRateEnable.SetValue(true);
        camera_->AcquisitionFrameRateAbs.SetValue(frame_rate_hz);
        frame_rate_hz = camera_->AcquisitionFrameRateAbs.GetValue();
        ROS_INFO("Framerate of camera '%s' set to  %.2f fps.", camera_name_.c_str(), frame_rate_hz);
        return true;
      }
      catch(std::exception & exception) {
        ROS_ERROR("An error occured while setting the framerate of camera '%s'.", camera_name_.c_str());
        return false;
      }
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::startStreaming() {
    if(camera_is_initialised_) {
#if !defined(CAMERA_IMAGE_QUEUE_SIZE) || CAMERA_IMAGE_QUEUE_SIZE == 1
      camera_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
#else
      camera_->OutputQueueSize = CAMERA_IMAGE_QUEUE_SIZE;
      camera_->StartGrabbing(Pylon::GrabStrategy_LatestImages);
#endif
      camera_is_streaming_ = true;
      ROS_INFO("Camera '%s' started grabbing and streaming.", camera_name_.c_str());
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::stopStreaming() {
    if(camera_is_initialised_) {
      camera_->StopGrabbing();
			camera_is_streaming_ = false;
      ROS_INFO("Camera '%s' stopped grabbing and streaming.", camera_name_.c_str());
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isStreaming() {
    if(camera_is_initialised_) {
      return camera_->IsGrabbing();
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::getImage(cv::Mat & image, ImageInfo & image_info) {
    if(camera_is_initialised_) {
      Pylon::CGrabResultPtr ptrGrabResult;
      Pylon::CPylonImage pylonImage;
      try {
        camera_->RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if(ptrGrabResult->GrabSucceeded()) {
          ros::Time timestamp = ros::Time::now() - image_transmission_time_;
          format_converter_.Convert(pylonImage, ptrGrabResult);
          image = cv::Mat(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3, pylonImage.GetBuffer()).clone();
          if(image.empty()) {
            ROS_WARN("Basler Pylon image converted to empty OpenCV image.");
            return false;
          } else {
            ROS_DEBUG("Basler Pylon image converted to OpenCV image.");
            // Estimate light level.
            double ev = std::numeric_limits<double>::quiet_NaN();
            double lux = std::numeric_limits<double>::quiet_NaN();
            // Send current camera settings to 'settings_debug' topic.
            image_info.header.stamp = timestamp;
            image_info.exposure_us = camera_->ExposureTimeAbs.GetValue();
            image_info.gain_db = camera_->GainRaw.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Red);
            image_info.wb_red = camera_->BalanceRatioAbs.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Green);
            image_info.wb_green = camera_->BalanceRatioAbs.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_GigECameraParams::BalanceRatioSelector_Blue);
            image_info.wb_blue = camera_->BalanceRatioAbs.GetValue();
            image_info.estimated_illuminance_lux = lux;
            return true;
          }
        } else {
          ROS_WARN("Basler Pylon grab failed. Ignoring frame.");
          return false;
        }
      }
      catch (GenICam::GenericException & exception) {
        ROS_ERROR("Received GenICam exception: \"%s\". Ignoring frame.", exception.what());
				// TODO: do we want to count these errors and crash/terminate if too many of them happen without a successful
				// grab in between?
        return false;
      }
      catch (...) {
        ROS_FATAL("Received unexpected exception while retrieving frame. Rethrowing.");
        throw;
        return false;
      }
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isCameraAttached() {
    if(camera_is_initialised_) {
      // return !camera_->IsCameraDeviceRemoved(); // TODO: No idea why this always returns false...
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonGigECameraAdapter::isCameraRemoved() {
    if(camera_is_initialised_) {
      // return camera_->IsCameraDeviceRemoved(); // TODO: No idea why this always returns true...
      return false;
    }
    else {
      return false;
    }
  }

}
