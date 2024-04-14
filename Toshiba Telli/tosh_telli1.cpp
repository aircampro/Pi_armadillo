// example using Toshiba Telli SDK for setting camera and saving frame and analysing with pekat
//
// get TelliCam driver from here https://www.toshiba-teli.co.jp/products/industrial-camera/software-telicamsdk.htm
// for rasberry pi it is arm version
// This software development kit is compatible with USB3 cameras, GigE cameras and CoaXPress cameras
//
#ifdef _WIN32
    // Don't warn about fopen_s
    #define _CRT_SECURE_NO_WARNINGS
    #include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <TeliCamApi.h>
#include <TeliCamUtl.h>
#include <pekatvision/sdk.h>

using namespace Teli;

int main(int argc, char* argv[]) {
    CAM_API_STATUS status;
    CAM_HANDLE hCam = (CAM_HANDLE)NULL;
    SIGNAL_HANDLE hSignal = (SIGNAL_HANDLE)NULL;
    CAM_STRM_HANDLE hStream = (CAM_STRM_HANDLE)NULL;
    void *buffer = NULL;
    void *image = NULL;
    pv_analyzer *pekat = NULL;
    uint32_t cameras;

    status = Sys_Initialize();
    if (status) {
        printf("Sys_Initialize error: 0x%08x\n", status);
        return 1;
    }

    do {
        status = Sys_GetNumOfCameras(&cameras);
        if (status) {
            printf("Sys_GetNumOfCameras error: 0x%08x\n", status);
            break;
        }
        printf("Number of cameras: %d\n", cameras);

        // for each camera on the link print its information
        for (int i = 0; i < cameras; i++) {
            CAM_INFO info;

            status = Cam_GetInformation((CAM_HANDLE)NULL, i, &info);
            if (status) {
                printf("Cam_GetInformation error: 0x%08x\n", status);
                break;
            }

            printf("Camera id = %d\n", i);
            printf("  Camera type: %s\n", info.eCamType == CAM_TYPE_U3V ? "USB3" : info.eCamType == CAM_TYPE_GEV ? "GigE" : "unknown");
            printf("  Manufacturer: %s\n", info.szManufacturer);
            printf("  Model: %s\n", info.szModelName);
            printf("  Serial no: %s\n", info.szSerialNumber);
            printf("  User name: %s\n", info.szUserDefinedName);
        }

        // if camera is found then choose which one you want 
        if (cameras) {
            uint32_t maxBufSize;

            // choose the camera ID from the list printed above
            int ia;
            std::cout << "Please enter the camera id to choose = ";
            std::cin >> ia;
            if (ia >= cameras) {
                std::cout << "\n The number entered is " << ia << " must be less than " << cameras << " index starts at 0 for frist camera " << "\n";
                std::cout << "Please enter the camera id to choose = ";
                std::cin >> ia;
                if (ia > cameras) { 
                    std::err << "\n The number entered is " << ia << " must be less than " << cameras << " index starts at 0 for frist camera " << "\033[31m Aborted \033[0m" << "\n";
                    return 1;
                }
            }
            
            // Create Pekat analyzer
            pekat = pv_create_remote_analyzer("127.0.0.1", 8100, NULL);
            if (!pekat) {
                printf("Failed to create Pekat analyzer\n");
                break;
            }
            
            // Open camera with ID=ia 
            status = Cam_Open(ia, &hCam);
            if (status) {
                printf("Cam_Open error: 0x%08x\n", status);
                break;
            }

            // Here you could set various properties using SDK

            // example set the exposure time to 500
            // CAMAPI GenApi_SetFloatValue(CAM_HANDLE hCam, const char *pszFeatureName, float64_t dValue, bool8_t bVerify = true);
            //
            const char * exp_tm = "ExposureTime";
            double exp_val = 500.0f;                                 // default exposure time is 500
            if (argc >= 1) {                                         // pass the exposure time as the first argument as an option
                try {
                    exp_val = atof(argv[1]);
                } catch {
                    std::err < "first argument is exposure time so it must be a floating point number using default 500.0" << std::endl;
                }
            }
            status = GenApi_SetFloatValue(hCam, exp_tm, exp_val, 1);
            if (status) {
                printf("GenApi_SetFloatValue for \"ExposureTime\" error: 0x%08x\n", status);
                break;
            }

            // example set the test pattern to ColorBar
            // CAMAPI GenApi_SetEnumStrValue(CAM_HANDLE hCam, const char *pszFeatureName, const char *pszBuf, bool8_t bVerify = true);
            //
            const char * featureNm = "TestPattern";
            const char * method = "ColorBar";
            status = GenApi_SetEnumStrValue(hCam, featureNm, method, 1);
            if (status) {
                printf("GenApi_SetFloatValue for \"TestPattern\" error: 0x%08x\n", status);
                break;
            }

            // example set AntiGlitch and AntiChatter parameters
            // large value means better noise immunity
            // you could measure the noise on the trigger signal with an oscilliscope
            //            
            double ValAntiGlitch = 0.0f;
            status = CAM_API_STS_SUCCESS;
            status = GenApi_GetFloatValue(hCam, "AntiGlitch", &ValAntiGlitch);
            if (status) {
                printf("GenApi_GetFloatValue for \"AntiGlitch\" error: 0x%08x\n", status);
                break;
            } else {
                printf("GenApi_GetFloatValue for \"AntiGlitch\" status: 0x%08x value= %f\n", status, ValAntiGlitch);  
                ValAntiGlitch = 1.1f;                                    // default integration time
                if (argc >= 2) {                                         // pass the anti-glitch (secs) as the 2nd argument as an option
                    try {
                        ValAntiGlitch = atof(argv[2]);
                    } catch {
                        std::err < "second argument is anti-glitch so it must be a floating point number using default 1.1s" << std::endl;
                    }
                }
                status = GenApi_SetFloatValue(hCam, "AntiGlitch", ValAntiGlitch, 1);    
                if (status) {
                    printf("GenApi_SetFloatValue for \"AntiGlitch\" error: 0x%08x\", status);
                    break;
                }                
            }                
            double ValAntiChat = 0.0f;
            status = CAM_API_STS_SUCCESS;
            status = GenApi_GetFloatValue(hCam, "AntiChattering", &ValAntiChat);
            if (status) {
                printf("GenApi_GetFloatValue for \"AntiChatter\" error: 0x%08x\n", status);
                break;
            } else {
                printf("GenApi_GetFloatValue for \"AntiChatter\" status: 0x%08x value= %f\n", status, ValAntiChat);  
                ValAntiChat = 0.45f;                                     // default insensitive time in seconds
                if (argc >= 3) {                                         // pass the anti-chatter (secs) as the 3rd argument as an option
                    try {
                        ValAntiChat = atof(argv[3]);
                    } catch {
                        std::err < "third argument is anti-cxhatter so it must be a floating point number using default 0.45s" << std::endl;
                    }
                }
                status = GenApi_SetFloatValue(hCam, "AntiChattering", ValAntiChat, 1);    
                if (status) {
                    printf("GenApi_SetFloatValue for \"AntiChatter\" error: 0x%08x\", status);
                    break;
                }                
            }
                       
            // Create signal
            status = Sys_CreateSignal(&hSignal);
            if (status) {
                printf("Sys_CreateSignal error: 0x%08x\n", status);
                break;
            }
            // Open stream
            status = Strm_OpenSimple(hCam, &hStream, &maxBufSize, hSignal);
            if (status) {
                printf("Strm_OpenSimple error: 0x%08x\n", status);
                break;
            }
            // Prepare buffer
            buffer = malloc(maxBufSize);
            if (!buffer) {
                printf("Cannot allocate %d bytes of memory\n", maxBufSize);
                break;
            }
            // Start streaming
            status = Strm_Start(hStream);
            if (status) {
                printf("Strm_Start error: 0x%08x\n", status);
                break;
            }
            // Wait for image
            status = Sys_WaitForSignal(hSignal, 2000);
            if (status == CAM_API_STS_SUCCESS) {
                // Image ready
                CAM_IMAGE_INFO imageInfo;
                uint32_t bufSize = maxBufSize;

                // Get image
                status = Strm_ReadCurrentImage(hStream, buffer, &bufSize, &imageInfo);
                if (status) {
                    printf("Strm_ReadCurrentImage error: 0x%08x\n", status);
                    break;
                }
                // Allocate buffer for RGB image
                image = malloc(imageInfo.uiSizeX * imageInfo.uiSizeY * 3);
                if (!image) {
                    printf("Cannot allocate memory for %dx%d image\n", imageInfo.uiSizeX, imageInfo.uiSizeY);
                    break;
                }
                // Convert to RGB
                status = ConvImage(DST_FMT_BGR24, imageInfo.uiPixelFormat, true, image, buffer, imageInfo.uiSizeX, imageInfo.uiSizeY);
                if (status) {
                    printf("ConvImage error: 0x%08x\n", status);
                    break;
                }
                // Save as bitmap
                status = SaveBmpRGB(image, imageInfo.uiSizeX, imageInfo.uiSizeY, "pict-orig.bmp");
                if (status) {
                    printf("SaveBmpRGB error: 0x%08x\n", status);
                    break;
                }
                // Analyze by Pekat
                int res = pv_analyze_raw_image(pekat, (char*)image, imageInfo.uiSizeX, imageInfo.uiSizeY, PVRT_ANNOTATED_IMAGE, NULL);
                if (res) {
                    printf("Analysis in Pekat failed: %d\n", res);
                    break;
                } else {
                    char *res_image = pv_get_result_data(pekat);
                    int res_size = pv_get_result_data_size(pekat);
                    FILE *f = fopen("pict-annotated.png", "wb");
                    if (f) {
                        fwrite(res_image, res_size, 1, f);
                        fclose(f);
                    } else {
                        printf("Cannot save Pekat result image to disk\n");
                    }
                }
            } else if (status == CAM_API_STS_TIMEOUT) {
                // Not received
                printf("Image not received in time\n");
            } else {
                printf("Sys_WaitForSignal error: 0x%08x\n", status);
                break;
            }
        }
    } while (0);

    // Cleanup
    if (hStream) {
        Strm_Stop(hStream);
        Strm_Close(hStream);
    }
    if (hSignal) {
        Sys_CloseSignal(hSignal);
    }
    if (hCam) {
        Cam_Close(hCam);
    }
    if (image) {
        free(image);
    }
    if (buffer) {
        free(buffer);
    }
    if (pekat) {
        pv_free_analyzer(pekat);
    }
    Sys_Terminate();
    return status ? 1 : 0;
}
