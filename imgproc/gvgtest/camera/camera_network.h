#ifndef CAMERA_NETWORK_H_
#define CAMERA_NETWORK_H_

#include "camera.h"
#include "camera_network_device.h"
#include "robot.h"

class CameraNetwork : public Camera {
public:
	CameraNetwork();
	virtual ~CameraNetwork() {};

	virtual bool openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight);
	virtual void closeCamera() {}; // TODO close device
	virtual bool isOpen();

	/// capture an image
	virtual bool capture();
	virtual CameraImage* createImage();

	/// set a camera setting
	virtual void setSetting(CAMERA_SETTING setting, int32_t value);

	/// get a camera setting
	virtual int32_t getSetting(CAMERA_SETTING setting);


protected:
	virtual bool initDevice(uint16_t requestedImageWidth, uint16_t requestedImageHeight);
	virtual void uninitDevice();
	virtual bool readFrame();

	CameraNetworkDevice *device;
};

#endif /* CAMERA_NETWORK_H_ */
