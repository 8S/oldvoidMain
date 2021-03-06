#ifndef VERDEX // do not include on 2009 robots

/** @file
 **
 ** Base class for Net GmbH's iCube camera, based on the Micron MT9P001.
 **
 ** TODO:
 **  - understand/document control values to get/set camera settings
 **  - initialization of camera settings
 **  - setting correct image size (one we want AND the camera can deliver)
 **  - ...
 **
 ***************************************************************************
 ***************************************************************************
 ** In order to use the iCube camera on Linux without being root, follow
 ** these instructions with the camera UNPLUGGED:
 **
 **  1. As root, create /etc/udev/rules.d/50-icube.rules and enter the
 **     the following text:
 **
 **     ATTRS{idVendor}=="152a", ATTRS{idProduct}=="8350", MODE="664", GROUP="plugdev"
 **
 **  2. Restart udev via "/etc/init.d/udev restart" (should not be necessary)
 **  3. Done.
 **
 **  Make sure by checking /etc/group that the plugdev group exists and you
 **  are a member of it (should be the default case for Ubuntu installations)
 **
 */

#include "camera_icube.h"
#include "camera_image.h"
#include "comm.h"
#include "robot.h"
#include "vision/image.h"
#include "camera_imageBayer.h"

#include <libusb-1.0/libusb.h>


/*------------------------------------------------------------------------------------------------*/

#define DEVICE_VID  0x152a
#define DEVICE_PID  0x8350

#define EP_INTR			(1 | LIBUSB_ENDPOINT_IN)
#define EP_DATA			(2 | LIBUSB_ENDPOINT_IN)
#define CTRL_IN			(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define USB_RQ			0x04
#define INTR_LENGTH		64


/*------------------------------------------------------------------------------------------------*/

/** Constructor.
 **
 ** Initializes class members and allocates buffers.
 */

CameraICube::CameraICube()
	: handle(0)
	, captureBufferIndex(0)
	, exportedBufferIndex(0)
	, buffer(0)
	, bufferSize(0x4C000) // will be used for data package size, best set it so that the last packet in a frame is a bit smaller
{
	cs.setName("iCube camera");

	buffer = (uint8_t*)malloc(bufferSize);

	for (int i = 0; i < BufferCount; i++) {
		imageBuffer[i].maxDataSize = 1280*960; // currently used maximum image resolution, theoretical max size is 2560*1920;
		imageBuffer[i].data = (char*)malloc(imageBuffer[i].maxDataSize);
	}

	//                  Settings name         Register name                  Label                  Default    Min    Max
	//---------------------------------------------------------------------------------------------------------------------
	addSupportedSetting(CAMERA_R_GAIN,        P_MT9X001_RED_GAIN,            "Red gain (analog)",        48,     0,    63); // high res 24
	addSupportedSetting(CAMERA_R_GAIN_A_MUL,  P_MT9X001_RED_GAIN,            "Red Gain x2 (a)",           1,     0,     1); // high res 1
	addSupportedSetting(CAMERA_R_GAIN_D,      P_MT9X001_RED_GAIN,            "Red Gain (digital)",        0,     0,   127); // high res 0

	addSupportedSetting(CAMERA_G_GAIN,        P_MT9X001_GREEN1_GAIN,         "Green 1 gain (analog)",    40,     0,    63); // high res 20
	addSupportedSetting(CAMERA_G1_GAIN_A_MUL, P_MT9X001_GREEN1_GAIN,         "Green 1 Gain x2 (a)",       1,     0,     1); // high res 1
	addSupportedSetting(CAMERA_G1_GAIN_D,     P_MT9X001_GREEN1_GAIN,         "Green 1 Gain (digital)",    0,     0,   127); // high res 0

	addSupportedSetting(CAMERA_B_GAIN,        P_MT9X001_BLUE_GAIN,           "Blue gain (analog)",       63,     0,    63); // high res 32
	addSupportedSetting(CAMERA_B_GAIN_A_MUL,  P_MT9X001_BLUE_GAIN,           "Blue gain x2 (a)",          1,     0,     1); // high res 1
	addSupportedSetting(CAMERA_B_GAIN_D,      P_MT9X001_BLUE_GAIN,           "Blue gain (digital)",       0,     0,   127); // high res 0

	addSupportedSetting(CAMERA_G2_GAIN,       P_MT9X001_GREEN2_GAIN,         "Green 2 gain (analog)",    40,     0,    63); // high res 20
	addSupportedSetting(CAMERA_G2_GAIN_A_MUL, P_MT9X001_GREEN2_GAIN,         "Green 2 gain x2 (a)",       1,     0,     1); // high res 1
	addSupportedSetting(CAMERA_G2_GAIN_D,     P_MT9X001_GREEN2_GAIN,         "Green 2 gain (digital)",    0,     0,   127); // high res 0

	addSupportedSetting(CAMERA_EXPOSURE,      P_MT9X001_SHUTTER_WIDTH_LOWER, "Shutter width",           120,     1, 65535); // high res 300

	addSupportedSetting(CAMERA_BLC_MANUAL,    P_MT9X001_BLACK_LEVEL_CALIBRATION, "Manual BLC",            0,     0,     1); // high res 1

	addSupportedSetting(CAMERA_GREEN1_OFFSET, P_MT9X001_GREEN1_OFFSET,       "Green1 offset",            20,  -255,   255); // high res -80
	addSupportedSetting(CAMERA_GREEN2_OFFSET, P_MT9X001_GREEN2_OFFSET,       "Green1 offset",            20,  -255,   255); // high res -80
	addSupportedSetting(CAMERA_RED_OFFSET,    P_MT9X001_RED_OFFSET,          "Red offset",               20,  -255,   255); // high res -80
	addSupportedSetting(CAMERA_BLUE_OFFSET,   P_MT9X001_BLUE_OFFSET,         "Blue offset",              20,  -255,   255); // high res -80

//	addSupportedSetting(CAMERA_VFLIP_IMAGE,   0,                             "Flip image (vert.)",        0,     0,     1);
//	addSupportedSetting(CAMERA_HFLIP_IMAGE,   0,                             "Flip image (horiz.)",       0,     0,     1);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 **/

CameraICube::~CameraICube() {
	if (isRunning())
		cancel(true);

	if (isOpen())
		closeCamera();
}


/*------------------------------------------------------------------------------------------------*/

/** Opens the connection to the camera.
 **
 ** @param deviceName            Name of device to use
 ** @param requestedImageWidth   Preferred width of image
 ** @param requestedImageHeight  Preferred height of image
 **
 ** @return true iff camera could be connected successfully
 **/

bool CameraICube::openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight) {
	if (strcmp(IMAGETYPENAME, "CameraImageBayer") != 0) {
		ERROR("Can not use iCube camera with non-Bayer image class %s! Adjust image.h!", IMAGETYPENAME);
		return false;
	}

	// as we will interpolate the image, fetch four times the size
#ifdef INTERPOLATE_BAYER
	imageWidth  = 2*requestedImageWidth;
	imageHeight = 2*requestedImageHeight;
#else
	imageWidth  = requestedImageWidth;
	imageHeight = requestedImageHeight;
#endif
	image = new CameraImageBayer(requestedImageWidth, requestedImageHeight);

	int ret = libusb_init(NULL);
	if (ret < 0) {
		ERROR("Could not initialize libusb, won't be able to connect to camera");
		return false;
	}

	handle = libusb_open_device_with_vid_pid(NULL, DEVICE_VID, DEVICE_PID);
	if (!handle) {
		ERROR("Could not open camera device 0x%x 0x%x", DEVICE_VID, DEVICE_PID);
		libusb_exit(NULL);
		return false;
	}

	libusb_detach_kernel_driver(handle, 0);

	ret = libusb_claim_interface(handle, 0);
	if (ret < 0) {
		ERROR("Could not claim interface for camera");
		closeCamera();
		return false;
	}

	// reset chip
	write_register(P_MT9X001_RESET, 1);
	delay(50);
	write_register(P_MT9X001_RESET, 0);

	// initialize camera
	init_camera();

	INFO("iCube camera connection successfully established");
	run();
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Sets initial configuration for the camera.
 **
 **/

bool CameraICube::init_camera() {
	int ret = 0;

	int horizontalStep = 2560 / imageWidth  - 1;
	int verticalStep   = 1920 / imageHeight - 1;

	write_register(P_MT9X001_ROW_ADDRESS_MODE,        verticalStep   | (verticalStep   << 4));
	write_register(P_MT9X001_COLUMN_ADDRESS_MODE,     horizontalStep | (horizontalStep << 4));
	write_register(P_MT9X001_COLUMN_SIZE,             0x09FF);
	write_register(P_MT9X001_ROW_SIZE,                0x077F);
	write_register(P_MT9X001_ROW_START,               0x0036);
	write_register(P_MT9X001_COLUMN_START,            0x0010);

	usleep(54*1000);

	// set active setting on interface
	unsigned char buf[1000];
	ret = libusb_get_descriptor(handle, 0x0000002, 0x0000000, buf, 0x0000009);
	usleep(1*1000);
	ret = libusb_get_descriptor(handle, 0x0000002, 0x0000000, buf, 0x0000087);
	ret = libusb_release_interface(handle, 0);
	if (ret != 0) ERROR("iCube - failed to release interface before set_configuration: %d", ret);
	ret = libusb_set_configuration(handle, 0x0000001);
	if (ret != 0) ERROR("iCube - Set configuration returned %d", ret);
	ret = libusb_claim_interface(handle, 0);
	if (ret != 0) ERROR("iCube - claim after set_configuration failed with error %d", ret);
	ret = libusb_set_interface_alt_setting(handle, 0, 0);
	if (ret != 0) ERROR("iCube - Set alternate setting returned %d", ret);
	usleep(18*1000);


	write_register(P_MT9X001_SHUTTER_DELAY,           0x000D);
	write_register(P_MT9X001_UNKNOWN_0x42,            0x0004);
	write_register(P_MT9X001_UNKNOWN_0x41,            0x0023);

	write_register(P_MT9X001_READ_MODE_1,             0x4006);
	write_register(P_MT9X001_READ_MODE_2,             0x0020);

	// no idea what this is doing
	buf[0] = 0x14;
	ret = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x00b8, 0, 0, buf, 1, 1000);
	usleep(125*1000);
	ret = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE + LIBUSB_ENDPOINT_IN, 0x00c5, 0, 0, buf, 1, 1000);
	usleep(101*1000);
	buf[0] = 0;
	ret = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x00b7, 0, 0, buf, 1, 1000);
	usleep(21*1000);
	ret = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE + LIBUSB_ENDPOINT_IN, 0x00c5, 0, 0, buf, 1, 1000);


	write_register(P_MT9X001_ROW_BLACK_TARGET,             0);
	write_register(P_MT9X001_HORIZONTAL_BLANKING,     0x01F4);

	// set test mode
	//write_register(P_MT9X001_TEST, (5 << 3) + 0x1);

	// Set the default configuration. This goes through the added settings (see
	// constructor) and sets the defined default values. Later on the configuration
	// file will be read and the default values may be changed again.
	for (uint32_t i=0; i < supportedSettings.size(); i++)
		setSetting(supportedSettings[i].setting, supportedSettings[i].defaultValue);


	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Closes the camera.
 **
 **/

void CameraICube::closeCamera() {
	if (handle) {
		libusb_set_interface_alt_setting(handle, 0, 0);
		libusb_attach_kernel_driver(handle, 0);
		libusb_close(handle);

		libusb_exit(NULL);
	}

	handle = 0;

	// freeing the buffer causes error "munmap_chunk(): invalid pointer: 0x..."
//	if (buffer)
//		free(buffer);
	buffer = 0;
}


/*------------------------------------------------------------------------------------------------*/

/** Checks whether connection to camera is established.
 **
 ** @return true iff camera is connected/on/open
 **/

bool CameraICube::isOpen() {
	return handle;
}



/*------------------------------------------------------------------------------------------------*/

/** USB callback
 **
 **
 */

void CameraICube::usb_callback(struct libusb_transfer *transfer) {
	// pass on to camera object
	ICubeUserData* userData = (ICubeUserData*)transfer->user_data;
	userData->cam->handle_usb_callback(transfer, userData->index);
}

void CameraICube::handle_usb_callback(struct libusb_transfer *transfer, int transferIndex) {

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		ICubeImageBuffer &imgBuf = imageBuffer[captureBufferIndex];
		if (imgBuf.dataLength == 0) imgBuf.startCapture = getCurrentTime();

		int remainingSpace = imgBuf.maxDataSize - imgBuf.dataLength;
		memcpy(imgBuf.data + imgBuf.dataLength, transfer->buffer, MIN(transfer->actual_length, remainingSpace));
		imgBuf.dataLength += transfer->actual_length;

//		printf("got %d bytes, %d total in buffer %d, request %d\n", transfer->actual_length, imgBuf.dataLength, captureBufferIndex, transferIndex);

		bool frameComplete  = false;
		bool frameCorrupted = false;

		if (transfer->actual_length < bufferSize /* indication that image is complete */) {
//			if (getCurrentTime() - imgBuf.startCapture > X) {
//				frameCorrupted = true;
//				printf("--> frame took too long (buffer %d)\n", captureBufferIndex);
//			} else
			if (imgBuf.dataLength >= imageWidth*imageHeight) {
				frameComplete  = true; // all okay
//				printf("--> frame complete (buffer %d)\n", captureBufferIndex);
			} else {
				frameCorrupted = true; // we miss data
//				printf("frame corrupt, too small (buffer %d)\n", captureBufferIndex);
			}
		} else if (imgBuf.dataLength >= imageWidth*imageHeight) {
			frameCorrupted = true;
//			printf("--> frame corrupt, too big (buffer %d)\n", captureBufferIndex);
		}

		if (frameCorrupted) {
			// reset frame
			imgBuf.dataLength = 0;
//			printf("Frame corrupt.\n");
		} else if (frameComplete) {
			// select next buffer to use
			CriticalSectionLock lock(cs);
//			printf("----\n");
//			printf("captured buffer %d\n", captureBufferIndex);
			for (int i=0; i < BufferCount; i++) {
				int idx = (captureBufferIndex + i + 1) % BufferCount;
				if (imageBuffer[idx].active) {
					captureBufferIndex = idx;
					imageBuffer[idx].filled = false;
					imageBuffer[idx].dataLength = 0;
//					printf("clearing filled but active frame %d\n", idx);
					break;
				}
			}

			// mark image as finished
			imgBuf.filled = true;

			// raise signal that we got an image
//			printf("triggering captured buffer\n");
			frameCapturedEvent.trigger();
		}
	} else
		ERROR("Image buffer transfer status: %d (0x%x)", transfer->status, transfer->status);

	// get next chunk
	libusb_submit_transfer(img_transfer[transferIndex]);
}


/*------------------------------------------------------------------------------------------------*/

/** Update the frame.
 **
 */

bool CameraICube::capture() {
	// wait for frame to be captured (may exit right away if we have a frame buffered)
	frameCapturedEvent.wait();

	// clear event
	frameCapturedEvent.reset();

	bool foundBuffer = false;
	int oldImageBufferIdx = exportedBufferIndex;

	// lock down so we get no conflicts changing the buffers
	cs.lock();

	// get last frame captured
	for (int i=0; i < BufferCount; i++) {
		int idx = (captureBufferIndex + BufferCount - i - 1) % BufferCount;
		if (imageBuffer[idx].active && imageBuffer[idx].filled) {
			foundBuffer  = true;
			exportedBufferIndex = idx;
			imageBuffer[exportedBufferIndex].active = false;
//			printf("using buffer %d\n", exportedBufferIndex);
			break;
		}
	}

	if (foundBuffer == false) {
		ERROR("Could not find a new image buffered even though we got a capture event.");
		for (int i=0; i < BufferCount; i++) {
//			printf("buffer %d: active %s, filled %s\n", i, imageBuffer[i].active ? "yes" : "no", imageBuffer[i].filled ? "yes" : "no");
		}
	}

//	printf("released previous buffer %d\n", oldImageBufferIdx);
	imageBuffer[oldImageBufferIdx].filled = false;
	imageBuffer[oldImageBufferIdx].active = true;

	cs.leave();

	image->setImage(imageBuffer[exportedBufferIndex].data, imageBuffer[exportedBufferIndex].dataLength);

	totalFrames++;
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Thread main function, keeps the libusb event loop pumping.
 **
 */

void CameraICube::threadMain() {
	alloc_transfers();
	init_capture();

	while (isRunning()) {
		struct timeval tv = { 0, 250000 };
		int r = libusb_handle_events_timeout(NULL, &tv);
		if (r < 0) {
			ERROR("Error handling libusb events, code %d (0x%d)", r, r);
			delay(100);
//			break;
		}
	}

	dealloc_transfers();
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

bool CameraICube::alloc_transfers() {
	for (int i=0; i < ImageTransferCount; i++) {
		img_transfer[i] = libusb_alloc_transfer(0);
		if (!img_transfer[i])
			return false;

		ICubeUserData *userData = new ICubeUserData();
		userData->cam = this;
		userData->index = i;
		libusb_fill_bulk_transfer(img_transfer[i], handle, EP_DATA, buffer,
				bufferSize, CameraICube::usb_callback, userData, 4686);
	}

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraICube::dealloc_transfers() {
	for (int i=0; i < ImageTransferCount; i++) {
		libusb_cancel_transfer(img_transfer[i]);
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

bool CameraICube::init_capture() {
	int r;

	for (int i=0; i < ImageTransferCount; i++) {

		r = libusb_submit_transfer(img_transfer[i]);
		if (r < 0) {
			return false;
		}
	}
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Setup register
 **
 ** Actually I have no idea what this is doing, but one of those mysterious lines is sent prior
 ** to every read and write request, and removing them makes for some "funny" effects.
 **
 ** @param reg    Register to set up
 */
void CameraICube::register_setup(ICubeRegister reg) {
	switch(reg) {
	case P_MT9X001_SHUTTER_DELAY:
	case P_MT9X001_UNKNOWN_0x41:
	case P_MT9X001_UNKNOWN_0x42:
		libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x00000bf, 0x00000d2, 0x0000000, NULL, 0, 1000);
		break;

	default:
		libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x00000bf, 0x0000090, 0x0000100, NULL, 0, 1000);
		break;
	}

	usleep(1*1000); // TODO: is this really necessary?
}


/*------------------------------------------------------------------------------------------------*/

/** Write a camera register.
 **
 ** A register on the MT9P001 chip is 16 bit.
 **
 ** A typical write sequence on the chip's two-wire serial interface starts with the write
 ** address (slave address of the chip), followed by the register address, followed by the
 ** 16 bit data.
 **
 ** A '0' in the LSB of the slave address indicates write mode, and a '1' indicates read mode.
 */

bool CameraICube::write_register(ICubeRegister reg, uint16_t value, unsigned char* data, uint16_t dataLen) {
	register_setup(reg);

	int ret = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x00000b2, value, reg, data, dataLen, 1000);
//	printf("write_register (reg 0x%x, value %d / 0x%x) returned %d\n", reg, value, value, ret);

	return ret == 0;
}


/*------------------------------------------------------------------------------------------------*/

/** Read a camera register.
 **
 ** A register on the MT9P001 chip is 16 bit.
 **
 ** A typical read sequence on the chip's two-wire serial interface starts similar to a write
 ** sequence, by sending the slave address followed by the register address. Now a start bit
 ** is sent, followed by the read address.
 */

bool CameraICube::read_register(ICubeRegister reg, uint16_t* value) {
	register_setup(reg);

	int ret = libusb_control_transfer(handle,
			LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE + LIBUSB_ENDPOINT_IN,
			0x00000b3, 0, reg, (unsigned char*)value, 2, 1000);

	*value = ntohs(*value);
	return ret == 2;
}


/*------------------------------------------------------------------------------------------------*/

/** Calculate a gain value. (MT9P001 doc page 20/21)
 **
 ** Effective gain is (digitalGain/8 + 1) * (analogMultiplier ? 2 : 1) * (analog_gain/8)
 **
 ** @param digitalGain        Range 0..120 (effective value is digitalGain/8 + 1)
 ** @param analogMultiplier   Whether an additional analog gain of 2x will be applied
 ** @param analogGain         Analog gain (effective value is times 8), range 0..63
 **
 ** @return gain value for register
 */

uint16_t CameraICube::gainValue(uint8_t digitalGain, bool analogMultiplier, uint8_t analogGain) {
	if (analogGain > 63)
		analogGain = 63;
	if (digitalGain > 120)
		digitalGain = 120;

	return (digitalGain * 256) + (analogMultiplier ? 64 : 0) + (analogGain);
}


/*------------------------------------------------------------------------------------------------*/

/** Sets a camera setting.
 **
 ** @param setting  Setting to set
 ** @param value    Value to set
 */

void CameraICube::setSetting(CAMERA_SETTING setting, int32_t value) {
	uint16_t oldValue;

	int16_t index = getSettingIndex(setting);
	if (index == -1)
		return; // not supported

	switch(setting) {
		case CAMERA_VFLIP_IMAGE:
			read_register(P_MT9X001_READ_MODE_2, &oldValue);
			if (value)
				write_register(P_MT9X001_READ_MODE_2, oldValue | (1<<15));
			else
				write_register(P_MT9X001_READ_MODE_2, oldValue & ~(1<<15));
			break;

		case CAMERA_HFLIP_IMAGE:
			read_register(P_MT9X001_READ_MODE_2, &oldValue);
			if (value)
				write_register(P_MT9X001_READ_MODE_2, oldValue | (1<<14));
			else
				write_register(P_MT9X001_READ_MODE_2, oldValue & ~(1<<14));
			break;

		case CAMERA_R_GAIN:
		case CAMERA_G_GAIN:
		case CAMERA_G2_GAIN:
		case CAMERA_B_GAIN: {
			int16_t a_mul = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN_A_MUL + (setting - CAMERA_R_GAIN)));
			int16_t d_gain = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN_D + (setting - CAMERA_R_GAIN)));
			write_register((ICubeRegister)supportedSettings[index].id, gainValue(d_gain, a_mul, value));
			break;
		}

		case CAMERA_R_GAIN_A_MUL:
		case CAMERA_G1_GAIN_A_MUL:
		case CAMERA_G2_GAIN_A_MUL:
		case CAMERA_B_GAIN_A_MUL: {
			int16_t a_gain = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN + (setting - CAMERA_R_GAIN_A_MUL)));
			int16_t d_gain = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN_D + (setting - CAMERA_R_GAIN_A_MUL)));
			write_register((ICubeRegister)supportedSettings[index].id, gainValue(d_gain, value ? true : false, a_gain));
			break;
		}

		case CAMERA_R_GAIN_D:
		case CAMERA_G1_GAIN_D:
		case CAMERA_G2_GAIN_D:
		case CAMERA_B_GAIN_D: {
			int16_t a_mul = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN_A_MUL + (setting - CAMERA_R_GAIN_D)));
			int16_t a_gain = getSetting((CAMERA_SETTING)(CAMERA_R_GAIN + (setting - CAMERA_R_GAIN_D)));
			write_register((ICubeRegister)supportedSettings[index].id, gainValue(value, a_mul, a_gain));
			break;
		}

		case CAMERA_BLC_MANUAL:
			write_register((ICubeRegister)supportedSettings[index].id, ((getSetting(CAMERA_BLC_MANUAL) & (~value)) | value));
			break;

		default:
			write_register((ICubeRegister)supportedSettings[index].id, value);
			break;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Gets a camera setting. Should only be called for settings where supports() returned true.
 **
 ** @param setting  Setting to get
 ** @return value of setting, -1 on error
 */

int32_t CameraICube::getSetting(CAMERA_SETTING setting) {
	uint16_t value;

	int16_t index = getSettingIndex(setting);
	if (index == -1)
		return -1; // not supported

	switch(setting) {
		case CAMERA_VFLIP_IMAGE:
			if (read_register(P_MT9X001_READ_MODE_2, &value))
				return value & (1<<15) ? 1 : 0;
			break;

		case CAMERA_HFLIP_IMAGE:
			if (read_register(P_MT9X001_READ_MODE_2, &value))
				return value & (1<<14) ? 1 : 0;
			break;

		case CAMERA_R_GAIN:
		case CAMERA_B_GAIN:
		case CAMERA_G_GAIN:
		case CAMERA_G2_GAIN:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value & 0x003F;
			break;

		case CAMERA_R_GAIN_A_MUL:
		case CAMERA_B_GAIN_A_MUL:
		case CAMERA_G1_GAIN_A_MUL:
		case CAMERA_G2_GAIN_A_MUL:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value & 0x0040;
			break;

		case CAMERA_R_GAIN_D:
		case CAMERA_B_GAIN_D:
		case CAMERA_G1_GAIN_D:
		case CAMERA_G2_GAIN_D:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value & 0xFF00;
			break;

		case CAMERA_BLC_MANUAL:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value & 1;
			break;

		case CAMERA_RED_OFFSET:
		case CAMERA_GREEN1_OFFSET:
		case CAMERA_GREEN2_OFFSET:
		case CAMERA_BLUE_OFFSET:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value <= 255 ? value : - 512 + value;
			break;

		default:
			if (read_register((ICubeRegister)supportedSettings[index].id, &value))
				return value;
			break;
	}

	return -1;
}


#endif // #ifndef VERDEX
