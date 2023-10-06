/* 
Record a datacube with a PikaL or Pika XC2 and save it to disk in a format readable by ENVI or Spectronon.
*/
#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <iomanip>
#include <assert.h>
#include <cstdlib>
#include "resonon_imager_basler.h"
int main()
{
    const int LINE_COUNT = 100; //size of datacube to record
    const std::string filename = "Pika_basler_test.bil"; //location to save datacube
    std::string header_filename; // header filename is the same with '.hdr' appended, see below
    int framesize; //size of a single frame in records (number of elements in array)
    int cubesize; //size of complete datacube in records (number of elements in array)
    unsigned short * buffer; //buffer to hold image data
    char imager_type_buffer[32]; //buffer to hold the imager type string
    char serial_number_buffer[32]; //buffer to hold the serial number string
    bool free_buffer = false; //keep track of whether we've allocated memory that must be released
    try
    {
        // Initialize imager.
        Resonon::PikaBasler imager;
        imager.connect(); //Be prepared to catch exceptions if the imager is not physically connected to the computer
        // Set Camera properties
        imager.set_framerate(100.0); //Hz
        imager.set_integration_time(7.0); //milliseconds
        imager.set_gain(0.0); //dB
        // Poll the imager type string and serial number
        imager.get_imager_type(imager_type_buffer, 32);
        imager.get_serial_number(serial_number_buffer, 32);
        imager.set_spectral_bin(imager.get_min_spectral_bin());
        //Print out the imager status
        std::cout << "Imager Status:\n";
        std::cout << "--------------\n";
        std::cout << std::setw(18) << "Imager Type: "  << imager_type_buffer << "\n";
        std::cout << std::setw(18) << "Serial Number: " << serial_number_buffer << "\n";
        std::cout << std::setw(18) << "Framerate: " << imager.get_framerate();
        std::cout << " [min: " << imager.get_min_framerate();
        std::cout << ", max: " << imager.get_max_framerate() << "]" << "\n";
        std::cout << std::setw(18) << "Integration Time: "  << imager.get_integration_time();
        std::cout << " [min: " << imager.get_min_integration_time();
        std::cout << ", max: " << imager.get_max_integration_time() << "]" << "\n";
        std::cout << std::setw(18) << "Gain: " << imager.get_gain();
        std::cout << " [min: " << imager.get_min_gain();
        std::cout << ", max: " << imager.get_max_gain() << "]" << "\n";
        std::cout << std::setw(18) << "Bands: " << imager.get_band_count() << "\n";
        std::cout << std::setw(18) << "Samples: " << imager.get_sample_count() << "\n";
        //allocate memory for datacube
        framesize = imager.get_band_count() * imager.get_sample_count();
        assert (framesize * sizeof(unsigned short) == imager.get_frame_buffer_size_in_bytes());
        cubesize = framesize * LINE_COUNT;
        buffer = new unsigned short[cubesize];
        if (buffer == 0)
        {
            std::cerr << "Error: memory could not be allocated for datacube";
            exit(EXIT_FAILURE);
        }
        free_buffer = true; //if an exception occurs below make sure we free the just allocated block of memory
        // record the datacube
        std::cout << "\nRecording Datacube" << std::endl;
        imager.start();
        for (int i = 0; i < LINE_COUNT; i++)
        {
            imager.get_frame(&buffer[i * framesize]);
            std::cout << "Line " << i + 1 << " of " << LINE_COUNT << std::endl;
        }
        imager.stop();
        std::cout << "Recording Complete\nWriting Datacube to Disk" << std::endl;
        //write an ENVI compatible header file
        header_filename = filename + ".hdr";
        std::ofstream outfile(header_filename.c_str());
        outfile << "ENVI\n";
        outfile << "interleave = bil\n";
        outfile << "data type = 12\n";
        outfile << "bit depth = 12\n";
        outfile << "samples = " << imager.get_sample_count() << "\n";
        outfile << "bands = " << imager.get_band_count() << "\n";
        outfile << "lines = " << LINE_COUNT << "\n";
        outfile << "framerate = " << imager.get_framerate() << "\n";
        outfile << "shutter = " << imager.get_integration_time() << "\n";
        outfile << "gain = " << imager.get_gain() << "\n";
        outfile << "wavelength = {";
        outfile << std::setprecision(5);
        int start = imager.get_start_band();
        int end = imager.get_end_band();
        for(int i = start; i < end - 1; i++)
        {
            outfile << imager.get_wavelength_at_band(i) << ", ";
        }
        outfile << imager.get_wavelength_at_band(end - 1) << "}\n";
        outfile.close();
        //write data file
        std::ofstream cubefile;
        cubefile.open(filename.c_str(), std::ios::out | std::ios::binary);
        cubefile.write((const char*) buffer, cubesize * sizeof(unsigned short));
        cubefile.close();
        std::cout << "Done." << std::endl;
        // free allocated resources
        imager.disconnect();
        delete [] buffer;
    } catch (std::exception const & e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        if (free_buffer == true)
            delete [] buffer;
        exit(EXIT_FAILURE);
    }
    return EXIT_SUCCESS;
}