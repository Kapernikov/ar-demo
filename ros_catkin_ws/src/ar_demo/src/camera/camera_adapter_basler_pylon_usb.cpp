//! Unified camera node - Basler Pylon USB adapter.
/*!
 *  \file
 *
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "camera_adapter_basler_pylon_usb.hpp"


namespace ar_demo {

  BaslerPylonUsbCameraAdapter::BaslerPylonUsbCameraAdapter(std::shared_ptr<const Configuration> configuration):
		CameraAdapter(configuration),
    camera_is_initialised_{false},
    camera_name_{""},
    camera_identification_{""},
    camera_is_streaming_{false},
    pait_{},
    camera_{nullptr},
    format_converter_{} {
    ROS_INFO("Using Basler Pylon USB camera adapter.");
  }

  BaslerPylonUsbCameraAdapter::~BaslerPylonUsbCameraAdapter() {
    if(camera_) {
      delete camera_;
    }
  }

  bool BaslerPylonUsbCameraAdapter::initialise(const std::string & camera_name, const std::string & model,
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
      camera_ = new Pylon::CBaslerUsbInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(deviceInfoList[0]));
      // Setup camera.
      camera_->Open();
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
      ROS_ERROR("An error occured during the initialisation of the camera.");
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isInitialised() {
    return camera_is_initialised_;
  }

  std::string BaslerPylonUsbCameraAdapter::getCameraIdentification() {
    if(camera_is_initialised_) {
      return camera_identification_;
    }
    else {
      return "";
    }
  }

  bool BaslerPylonUsbCameraAdapter::setManualExposure(double time_s, double gain_db) {
    if(camera_is_initialised_) {
      // Exposure time.
      camera_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
      camera_->ExposureTime.SetValue(time_s * 1.e6);
      ROS_INFO("Exposure time: %.2f µs.", camera_->ExposureTime.GetValue());
      // Analog gain.
      camera_->GainAuto.SetValue(Basler_UsbCameraParams::GainAuto_Off);
      camera_->Gain.SetValue(gain_db);
      ROS_INFO("Gain: %.2f dB.", camera_->Gain.GetValue());
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isAutoExposureSupported() {
    return true;
  }

  bool BaslerPylonUsbCameraAdapter::setAutoExposure(double target_brightness, double time_s_min, double time_s_max,
      double gain_db_min, double gain_db_max) {
    if(camera_is_initialised_) {
      // Target brightness.
      try {
        camera_->AutoTargetBrightness.SetValue(target_brightness);
        target_brightness = camera_->AutoTargetBrightness.GetValue();
        ROS_INFO("Target brightness: %.4f.", target_brightness);
      }
      catch(...) {
        ROS_WARN("Failed to set requested target brightness %.4f. Current target brightness: %.4f.",
            target_brightness, camera_->AutoTargetBrightness.GetValue());
      }
      // Set exposure time
      if(time_s_min < camera_->AutoExposureTimeUpperLimit.GetValue() / 1.e6) {
        // If the new lower limit is smaller than the old upper limit, set the lower limit first.
        camera_->AutoExposureTimeLowerLimit.SetValue(time_s_min * 1.e6);
        camera_->AutoExposureTimeUpperLimit.SetValue(time_s_max * 1.e6);
      }
      else {
        // If the new lower limit is larger then the old upper limit, set the upper limit first.
        camera_->AutoExposureTimeUpperLimit.SetValue(time_s_max * 1.e6);
        camera_->AutoExposureTimeLowerLimit.SetValue(time_s_min * 1.e6);
      }
      camera_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
      ROS_INFO("Exposure time: %.3f s ... %.3f s.", time_s_min, time_s_max);
      // Set gain.
      if(gain_db_min < camera_->AutoGainUpperLimit.GetValue()) {
        camera_->AutoGainLowerLimit.SetValue(gain_db_min);
        camera_->AutoGainUpperLimit.SetValue(gain_db_max);
      }
      else {
        camera_->AutoGainUpperLimit.SetValue(gain_db_max);
        camera_->AutoGainLowerLimit.SetValue(gain_db_min);
      }
      camera_->GainAuto.SetValue(Basler_UsbCameraParams::GainAuto_Continuous);
      ROS_INFO("Analog gain: %.2f dB ... %.2f dB.", gain_db_min, gain_db_max);
      return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isAutoExposureRoiSupported() {
    return true;
  }

  bool BaslerPylonUsbCameraAdapter::setAutoExposureRoi(int column, int row, int width, int height) {
    if(camera_is_initialised_) {
      if(column == -1 && row == -1 && width == -1 && height == -1) {
        column = camera_->OffsetX.GetValue();
        row = camera_->OffsetY.GetValue();
        width = camera_->Width.GetValue();
        height = camera_->Height.GetValue();
      }
#if PYLON_VERSION_MAJOR <= 4
      camera_->AutoFunctionAOISelector.SetValue(Basler_UsbCameraParams::AutoFunctionAOISelector_AOI1);
      camera_->AutoFunctionAOIOffsetX.SetValue(0);
      camera_->AutoFunctionAOIOffsetY.SetValue(0);
      camera_->AutoFunctionAOIWidth.SetValue(width);
      camera_->AutoFunctionAOIHeight.SetValue(height);
      camera_->AutoFunctionAOIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionAOIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto brightness.
      camera_->AutoFunctionAOIUseBrightness.SetValue(true);
      camera_->AutoFunctionAOIUseWhiteBalance.SetValue(false);
#else // PYLON_VERSION_MAJOR == 5
      // Disable second ROI use
      camera_->AutoFunctionROISelector.SetValue(Basler_UsbCameraParams::AutoFunctionROISelector_ROI2);
      camera_->AutoFunctionROIUseBrightness.SetValue(false);
      // Enable first ROI for Intensity
      camera_->AutoFunctionROISelector.SetValue(Basler_UsbCameraParams::AutoFunctionROISelector_ROI1);
      camera_->AutoFunctionROIOffsetX.SetValue(0);
      camera_->AutoFunctionROIOffsetY.SetValue(0);
      camera_->AutoFunctionROIWidth.SetValue(width);
      camera_->AutoFunctionROIHeight.SetValue(height);
      camera_->AutoFunctionROIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionROIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto brightness.
      camera_->AutoFunctionROIUseBrightness.SetValue(true);
      camera_->AutoFunctionROIUseWhiteBalance.SetValue(false);
#endif
      ROS_INFO("Auto exposure ROI: (%d, %d) - (%d, %d).", column, row, column + width - 1, row + height -1);
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::setManualWhiteBalance(double red, double green, double blue) {
    if(camera_is_initialised_) {
      camera_->BalanceWhiteAuto.SetValue(Basler_UsbCameraParams::BalanceWhiteAuto_Off);
      // camera_->LightSourcePreset.SetValue(Basler_UsbCameraParams::LightSourcePreset_Tungsten2800K);
      // camera_->LightSourcePreset.SetValue(Basler_UsbCameraParams::LightSourcePreset_Daylight5000K);
      camera_->LightSourcePreset.SetValue(Basler_UsbCameraParams::LightSourcePreset_Off);
      camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Red);
      camera_->BalanceRatio.SetValue(red);
      camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Green);
      camera_->BalanceRatio.SetValue(green);
      camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Blue);
      camera_->BalanceRatio.SetValue(blue);
      ROS_INFO("White balance: R: %.2f, G: %.2f, B: %.2f.", red, green, blue);
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isAutoWhiteBalanceSupported() {
    return true;
  }

  bool BaslerPylonUsbCameraAdapter::setAutoWhiteBalance() {
    if(camera_is_initialised_) {
      camera_->LightSourcePreset.SetValue(Basler_UsbCameraParams::LightSourcePreset_Off);
      camera_->ColorTransformationSelector.SetValue(Basler_UsbCameraParams::ColorTransformationSelector_RGBtoRGB);
      camera_->BalanceWhiteAuto.SetValue(Basler_UsbCameraParams::BalanceWhiteAuto_Continuous);
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isAutoWhiteBalanceRoiSupported() {
    return true;
  }

  bool BaslerPylonUsbCameraAdapter::setAutoWhiteBalanceRoi(int column, int row, int width, int height) {
    if(camera_is_initialised_) {
      if(column == -1 && row == -1 && width == -1 && height == -1) {
        column = camera_->OffsetX.GetValue();
        row = camera_->OffsetY.GetValue();
        width = camera_->Width.GetValue();
        height = camera_->Height.GetValue();
      }
#if PYLON_VERSION_MAJOR <= 4
      camera_->AutoFunctionAOISelector.SetValue(Basler_UsbCameraParams::AutoFunctionAOISelector_AOI2);
      camera_->AutoFunctionAOIOffsetX.SetValue(0);
      camera_->AutoFunctionAOIOffsetY.SetValue(0);
      camera_->AutoFunctionAOIWidth.SetValue(width);
      camera_->AutoFunctionAOIHeight.SetValue(height);
      camera_->AutoFunctionAOIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionAOIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto white balance.
      camera_->AutoFunctionAOIUseWhiteBalance.SetValue(true);
      camera_->AutoFunctionAOIUseBrightness.SetValue(false);
#else // PYLON_VERSION_MAJOR == 5
      // Disable second ROI use
      camera_->AutoFunctionROISelector.SetValue(Basler_UsbCameraParams::AutoFunctionROISelector_ROI1);
      camera_->AutoFunctionROIUseWhiteBalance.SetValue(false);
      // Enable first ROI for Intensity
      camera_->AutoFunctionROISelector.SetValue(Basler_UsbCameraParams::AutoFunctionROISelector_ROI2);
      camera_->AutoFunctionROIOffsetX.SetValue(0);
      camera_->AutoFunctionROIOffsetY.SetValue(0);
      camera_->AutoFunctionROIWidth.SetValue(width);
      camera_->AutoFunctionROIHeight.SetValue(height);
      camera_->AutoFunctionROIOffsetX.SetValue(column + camera_->OffsetX.GetValue());
      camera_->AutoFunctionROIOffsetY.SetValue(row + camera_->OffsetY.GetValue());
      // Use for auto white balance.
      camera_->AutoFunctionROIUseWhiteBalance.SetValue(true);
      camera_->AutoFunctionROIUseBrightness.SetValue(false);
#endif
      ROS_INFO("Auto white balance ROI: (%d, %d) - (%d, %d).", column, row, column + width - 1, row + height -1);
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::setFrameRate(double frame_rate_hz) {
    if(camera_is_initialised_) {
      try {
        camera_->AcquisitionFrameRateEnable.SetValue(true);
        camera_->AcquisitionFrameRate.SetValue(frame_rate_hz);
        frame_rate_hz = camera_->AcquisitionFrameRate.GetValue();
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

  bool BaslerPylonUsbCameraAdapter::startStreaming() {
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

  bool BaslerPylonUsbCameraAdapter::stopStreaming() {
    if(camera_is_initialised_) {
      camera_->StopGrabbing();
      ROS_INFO("Camera '%s' stopped grabbing and streaming.", camera_name_.c_str());
			return true;
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isStreaming() {
    if(camera_is_initialised_) {
      return camera_->IsGrabbing();
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::getImage(cv::Mat & image, ImageInfo & image_info) {
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
            double ev = std::log2(pow(aperture_, 2)
                / (camera_->ExposureTime.GetValue() / 1.e+6 * camera_->AutoTargetBrightness.GetValue()))
              - (camera_->Gain.GetValue() / 6.) + ev_correction_;
            double lux = 2.5 * pow(2., ev);
            // Send current camera settings to 'settings_debug' topic.
            image_info.header.stamp = timestamp;
            image_info.exposure_us = camera_->ExposureTime.GetValue();
            image_info.gain_db = camera_->Gain.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Red);
            image_info.wb_red = camera_->BalanceRatio.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Green);
            image_info.wb_green = camera_->BalanceRatio.GetValue();
            camera_->BalanceRatioSelector.SetValue(Basler_UsbCameraParams::BalanceRatioSelector_Blue);
            image_info.wb_blue = camera_->BalanceRatio.GetValue();
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

  bool BaslerPylonUsbCameraAdapter::isCameraAttached() {
    if(camera_is_initialised_) {
      return !camera_->IsCameraDeviceRemoved();
    }
    else {
      return false;
    }
  }

  bool BaslerPylonUsbCameraAdapter::isCameraRemoved() {
    if(camera_is_initialised_) {
      return camera_->IsCameraDeviceRemoved();
    }
    else {
      return false;
    }
  }

}
