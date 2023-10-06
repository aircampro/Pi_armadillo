/*
Record a calibrated datacube (dark frame subtracted and response corrected) with a PikaL or Pika XC2 and save it to disk in a format readable by ENVI or Spectronon.
*/
#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <iomanip>
#include <assert.h>
#include <cstdlib>
#include "resonon_imager_basler.h"
int main() {
    const int LINE_COUNT = 100; //size of datacube to record
    const int DARK_COUNT = 30; //size of dark cube to record
    const int RESPONSE_COUNT = 25; //size of response cube to record
    const std::string filename = "Pika_basler_test.bil"; //location to save datacube
    std::string header_filename; // header filename is the same with '.hdr' appended, see below
    int framesize; //size of a single frame in records (number of elements in array)
    int cubesize; //size of complete datacube in records (number of elements in array)
    unsigned short * frame_buffer; //buffer to hold image data
    double * cube_buffer; //buffer to hold calibrated data cube
    double * dark_buffer; //buffer to hold dark cube
    double * response_buffer; //buffer to hold response cube
    double corrected_pixel; // store final calibrated value at given pixel
    try {
        // Initialize imager.
        Resonon::PikaBasler imager;
        imager.connect(); //Be prepared to catch exceptions if the imager is not physically connected to the computer
        // Set Camera properties
        imager.set_framerate(100.0); //Hz
        imager.set_integration_time(7.0); //milliseconds
        imager.set_gain(0.0); //dB
        //allocate memory for datacube
        framesize = imager.get_band_count() * imager.get_sample_count();
        assert (framesize * sizeof(unsigned short) == imager.get_frame_buffer_size_in_bytes());
        //
        // Set up frame buffer
        //
        frame_buffer = new unsigned short[framesize]();
        if (frame_buffer == 0) {
            throw std::exception("Error: memory could not be allocated for frame");
        } // end if
        //
        // Set up dark buffer, record dark cube
        //
        dark_buffer = new double[framesize]();
        if (dark_buffer == 0) {
            throw std::exception("Error: memory could not be allocated for dark cube");
        } // end if
        // record the dark cube
        std::cout << "\nPlace lens cap on camera to record Dark Cube\n";
        std::cout << "Press Enter to begin recording\n";
        std::cin.ignore(); // 'pause' before recording the dark cube
        std::cout << "\nRecording Dark Cube" << std::endl;
        imager.start();
        for (int line = 0; line < DARK_COUNT; line++) {
            imager.get_frame(frame_buffer);
            // Add each frame to dark_buffer
            for (int pixel = 0; pixel < framesize; pixel++) {
                dark_buffer[pixel] += double(frame_buffer[pixel]);
            } // end for
            std::cout << "Line " << line + 1 << " of " << DARK_COUNT << std::endl;
        } // end for
        imager.stop();
        // Average dark_buffer
        for (int k = 0; k < framesize; k++) {
            dark_buffer[k] /= DARK_COUNT;
        } //end for
        std::cout << "Dark Cube recording complete" << std::endl;
        //
        // Set up response buffer, record response cube
        //
        response_buffer = new double[framesize]();
        if (response_buffer == 0) {
            throw std::exception("Error: memory could not be allocated for response cube");
        } // end if
        // record the response cube
        std::cout << "\nPlace calibration panel in front of camera to record Response Cube\n";
        std::cout << "Press Enter to begin recording\n";
        std::cin.ignore(); // 'pause' before recording the response cube
        std::cout << "\nRecording Response Cube" << std::endl;
        imager.start();
        for (int line = 0; line < RESPONSE_COUNT; line++) {
            imager.get_frame(frame_buffer);
            // Add each frame to response_buffer
            for (int pixel = 0; pixel < framesize; pixel++) {
                response_buffer[pixel] += double(frame_buffer[pixel]);
            } // end for
            std::cout << "Line " << line + 1 << " of " << RESPONSE_COUNT << std::endl;
        } //end for
        imager.stop();
        // Average response_buffer
        for (int k = 0; k < framesize; k++) {
            response_buffer[k] /= RESPONSE_COUNT;
        } // end for
        std::cout << "Response Cube recording complete" << std::endl;
        //
        // Set up cube buffer, record datacube
        //
        cubesize = framesize * LINE_COUNT;
        cube_buffer = new double[cubesize]();
        if (cube_buffer == 0) {
            throw std::exception("Error: memory could not be allocated for datacube");
        } // end if
        // record the datacube
        std::cout << "\nAim camera at target to record Datacube\n";
        std::cout << "Press Enter to begin recording\n";
        std::cin.ignore(); // 'pause' before recording the response cube
        std::cout << "\nRecording Datacube" << std::endl;
        imager.start();
        for (int line = 0; line < LINE_COUNT; line++) {
            imager.get_frame(frame_buffer);
            // Record and calibrate Datacube
            for (int pixel = 0; pixel < framesize; pixel++) {
                corrected_pixel = (double(frame_buffer[pixel]) - dark_buffer[pixel]) / (response_buffer[pixel] - dark_buffer[pixel] + 1e-9); // small value added to avoid divide by 0 error
                if (corrected_pixel < 0) {
                    corrected_pixel = 0; // ensure final result isn't negative, should happen rarely
                } //end if
                // Alternate form: cube_buffer[line][pixel] = corrected_pixel *if* cube_buffer is instantiated and deleted as a 2D array
                cube_buffer[line * framesize + pixel] = corrected_pixel;
            } //end for
            std::cout << "Line " << line + 1 << " of " << LINE_COUNT << std::endl;
            
        } //end for
        imager.stop();  
        std::cout << "Recording Complete\nWriting Datacube to Disk" << std::endl;
        //write an ENVI compatible header file
        header_filename = filename + ".hdr";
        std::ofstream outfile(header_filename.c_str());
        outfile << "ENVI\n";
        outfile << "interleave = bil\n";
        outfile << "data type = 5\n"; //ENVI datatype representation is 5 for double precision values (https://www.l3harrisgeospatial.com/docs/enviheaderfiles.html)
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
        for(int band = start; band < end - 1; band++) {
            outfile << imager.get_wavelength_at_band(band) << ", ";
        } // end for
        outfile << imager.get_wavelength_at_band(end - 1) << "}\n";
        outfile.close();
        //write data file
        std::ofstream cubefile;
        cubefile.open(filename.c_str(), std::ios::out | std::ios::binary);
        cubefile.write((const char*) cube_buffer, cubesize * sizeof(double));
        cubefile.close();
        std::cout << "Done." << std::endl;
        // free allocated resources
        imager.disconnect();
        delete [] frame_buffer;
        delete [] dark_buffer;
        delete [] response_buffer;
        delete [] cube_buffer;
    } catch (std::exception const & e) {
        std::cerr << "Error: " << e.what() << std::endl;
        if (frame_buffer != 0) {
            delete [] frame_buffer;
        } // end if
        if (dark_buffer != 0) {
            delete [] dark_buffer;
        } // end if
        if (response_buffer != 0) {
            delete [] response_buffer;
        } // end if
        if (cube_buffer != 0) {
            delete [] cube_buffer;
        } // end if
        exit(EXIT_FAILURE);
    } // end try/catch
    return EXIT_SUCCESS;
} // end main
