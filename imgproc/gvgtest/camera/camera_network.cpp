/**
 ** Network camera
 **
 ** Protocol description:
 **  * communication is on port 31313
 **  *
 */

#include "camera_network.h"

#include "camera_imageBayer.h"
#include "vision/image.h"

#include "robot.h"


/*------------------------------------------------------------------------------------------------*/

/**
 */

CameraNetwork::CameraNetwork() {
	device = &(CameraNetworkDevice::getInstance());
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

bool CameraNetwork::openCamera(
		const char* deviceName,
		uint16_t    requestedImageWidth,
		uint16_t    requestedImageHeight)
{
	if (device->startCamera(requestedImageWidth, requestedImageHeight) > 0) {
		if (initDevice(requestedImageWidth, requestedImageHeight)) {
			defaultConfiguration();
			return true;
		} else {
			ERROR("initDevice failed: can't set image size");
			return false;
		}
	}

	ERROR("startCamera() failed");
	return false;
}


/*------------------------------------------------------------------------------------------------*/

/** Capture an image.
 **
 ** @return true if a (new) image has been captured
 */

bool CameraNetwork::capture() {
	if (readFrame()) {
		totalFrames++;
		return true;
	}

	return false;
}


/*------------------------------------------------------------------------------------------------*/

/** Read an image frame.
 **
 ** @return true if a new image has been captured
 */

bool CameraNetwork::readFrame() {
	if (device && device->isRunning() && device->frameCapturedEvent.wait(50)) {
		image->setImage(device->getBuffer(), device->getImageSize(), device->getWidth(), device->getHeight());
		return true;
	} else
		return false;
}


/*------------------------------------------------------------------------------------------------*/

/** Initialize camera device
 **
 ** @param requestedImageWidth    Image width in pixels
 ** @param requestedImageHeight   Image height in pixels
 **
 ** @return true if device was successfully initialized
 */

bool CameraNetwork::initDevice(uint16_t requestedImageWidth, uint16_t requestedImageHeight) {
	if (false == device->running())
		return false;

	// remember image dimensions
	imageWidth  = requestedImageWidth;
	imageHeight = requestedImageHeight;

	// Create CameraImage Object
	image = createImage();
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Create an image from the last capture
 */

CameraImage* CameraNetwork::createImage() {
	return new IMAGETYPE(imageWidth, imageHeight);
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

void CameraNetwork::uninitDevice() {

}


/*------------------------------------------------------------------------------------------------*/

/**
 */

void CameraNetwork::setSetting(CAMERA_SETTING setting, int32_t value) {
	supportedSettings[ getSettingIndex(setting) ].currentValue = value;

	device->setSetting(setting, value);
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

int32_t CameraNetwork::getSetting(CAMERA_SETTING setting) {
	int16_t index = getSettingIndex(setting);
	if (index >= 0)
		return device->getSetting(setting);
	else
		return -1;
}


/*------------------------------------------------------------------------------------------------*/

/** Check whether camera is still open.
 **
 ** @return true iff camera is still open
 */

bool CameraNetwork::isOpen() {
	return (device != 0 && device->isRunning());
}
