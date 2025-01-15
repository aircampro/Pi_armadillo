// This is an interface between the Raspi Camera on CSI and the LCD Display panel to view the video
//
// ref project :- https://github.com/iwatake2222/DigitalCamera_RaspberryPi/tree/master
//
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "common.h"
#include "util.h"
#include "CameraCtrl.hpp"
#include "Ili9341Spi.hpp"
#include "DdCamera.hpp"

CameraCtrl::CameraCtrl(uint32_t width, uint32_t height)
{
	m_width = width;
	m_height = height;
	m_ddCamera = NULL;
	if (m_config.Load(m_inipath) == 0) {
		LOG_E("could not load the .ini file %s\n",m_inipath);
    }  
	m_ddIli9341Spi = new Ili9341Spi(m_config.DD_ILI_9341_SPI_SPI_DEV, m_config.DD_ILI_9341_SPI_GPIO_DC);
}

CameraCtrl::~CameraCtrl()
{
	if(m_ddCamera) liveviewStop();	                                                 // in case liveview was not stopped
	delete m_ddIli9341Spi;
}

RET CameraCtrl::liveviewStart()
{	
	if(m_ddCamera) return RET_ERR;
	m_bufferRGB565 = new uint8_t[m_width * m_height * 2];
	m_bufferRGB888 = new uint8_t[m_width * m_height * 3];  
	m_bufferYUYV = new uint8_t[m_width * m_height * 4];	
	//m_ddCamera = new DdCamera(m_width, m_height, DdCamera::CAPTURE_FORMAT_RGB888);
	m_ddCamera = new DdCamera(m_width, m_height, m_config.CAP_FMT);                // read the capture format from the .ini file
	m_ddCamera->captureStart();
	return RET_OK;
}

RET CameraCtrl::liveviewStop()
{	
	if(!m_ddCamera) return RET_ERR;
	m_ddCamera->captureStop();
	delete m_ddCamera;
	delete[] m_bufferRGB565;
	delete[] m_bufferRGB888;
	delete[] m_bufferYUYV;
	m_ddCamera = NULL;
	return RET_OK;
}

// grab the frame in the format specified, convert and display it on the LCD
RET CameraCtrl::liveviewFrame()
{	
	uint32_t size;

    // grab the picture from the camera 
	if (m_config.CAP_FMT == DdCamera::CAPTURE_FORMAT_RGB888) { 
	    m_ddCamera->captureCopyBuffer(m_bufferRGB888, &size);
	    if (size != m_width * m_height * 3) {
		    LOG_E("size must be %d, but it was %d\n", m_width * m_height * 3, size);
		    return RET_ERR;
	    }
	    convertRGB888To565(m_bufferRGB888, m_bufferRGB565, m_width * m_height);	               // convert from 888 to 565
	} else if (m_config.CAP_FMT == DdCamera::CAPTURE_FORMAT_YUYV) { 
	    m_ddCamera->captureCopyBuffer(m_bufferYUYV, &size);
	    if (size != m_width * m_height * 4) {
		    LOG_E("size must be %d, but it was %d\n", m_width * m_height * 4, size);
		    return RET_ERR;
	    }
	    convertYUYVToRGB565(m_bufferYUYV, m_bufferRGB565, m_width * m_height);
	} else if (m_config.CAP_FMT == DdCamera::CAPTURE_FORMAT_YVYU) { 
	    m_ddCamera->captureCopyBuffer(m_bufferYUYV, &size);
	    if (size != m_width * m_height * 4) {
		    LOG_E("size must be %d, but it was %d\n", m_width * m_height * 4, size);
		    return RET_ERR;
	    }
	    convertYVYUToRGB565(m_bufferYUYV, m_bufferRGB565, m_width * m_height);
	} else if (m_config.CAP_FMT == DdCamera::CAPTURE_FORMAT_YUV420) { 
	    m_ddCamera->captureCopyBuffer(m_bufferYUYV, &size);
	    if (size != m_width * m_height * 4) {
		    LOG_E("size must be %d, but it was %d\n", m_width * m_height * 4, size);
		    return RET_ERR;
	    }
	    NV12ToRGB(m_bufferYUYV, m_bufferRGB888, m_width, m_height);                             // YUV420 Semi-planar which is NV12 to RGB888
	    convertRGB888To565(m_bufferRGB888, m_bufferRGB565, m_width * m_height);			
	}
    // draw the buffer on the display
	m_ddIli9341Spi->drawBuffer((uint16_t*)m_bufferRGB565);
	
	return RET_OK;
}


RET CameraCtrl::captureJpeg(const char* filename)
{	
	if(m_ddCamera) liveviewStop();
	uint8_t *bufferJpeg = new uint8_t[m_width * m_height * 2];
	m_ddCamera = new DdCamera(m_width, m_height, DdCamera::CAPTURE_FORMAT_JPG);
	m_ddCamera->captureStart();

	uint32_t size;
	m_ddCamera->captureCopyBuffer(bufferJpeg, &size);
	saveFileBinary(filename, bufferJpeg, size);

	m_ddCamera->captureStop();
	delete m_ddCamera;
	delete[] bufferJpeg;
	m_ddCamera = NULL;

	liveviewStart();	// restart liveview
	return RET_OK;
}