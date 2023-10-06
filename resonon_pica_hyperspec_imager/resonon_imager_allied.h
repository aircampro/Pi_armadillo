/*
  * Resonon API
  *
  * By using this API, the user agrees to the terms and conditions as stated in the
  * document "Resonon API Terms of Use", located on the Resonon website
  * at: http://www.resonon.com/downloads/Resonon_API_Terms_of_Use.pdf.
  *
  */
 
 #ifndef __GUARD_RESONON_IMAGER_ALLIED_H
 #define __GUARD_RESONON_IMAGER_ALLIED_H
 #include "resonon_imager_base.h"
 
 namespace Resonon
 {
     class AlliedCameraImplementation;
     class RESONONDLL PikaAllied : public ResononImagerBase
         {
             public:
                 PikaAllied();
                 virtual ~PikaAllied();
                 void connect(const char * camera_serial_number=NULL);
                 void disconnect();
                 void start();
                 void stop();
                 void get_imager_type(char *buffer, int buffer_size);
                 void get_serial_number(char *buffer, int buffer_size);
                 void get_camera_serial_number(char *buffer, int buffer_size);
                 void generate_configuration_report(char *buffer, int buffer_size);
                 float get_coeff_a();
                 float get_coeff_b();
                 float get_coeff_c();
                 double get_wavelength_at_band(const int band);
                 int get_frame_buffer_size_in_bytes();
                 unsigned short* get_frame(unsigned short* buffer);
                 std::uint64_t get_last_timestamp();
                 std::uint64_t ticks_per_second();
                 int get_band_count();
                 int get_sample_count();
                 void set_framerate(const double frames_per_second);
                 double get_framerate();
                 double get_min_framerate();
                 double get_max_framerate();
                 double get_min_integration_time();
                 double get_max_integration_time();
                 void set_integration_time(const double milliseconds);
                 double get_integration_time();
                 void set_internal_trigger();
                 void set_external_trigger(unsigned int signal_line, bool rising_edge=true);
                 bool is_trigger_external();
             protected:
                 AlliedCameraImplementation * pimpl;
         };
 
 } // end namespace Resonon
 #endif //end ifndef __GUARD_RESONON_IMAGER_ALLIED_H