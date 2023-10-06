 /*
  * Resonon API
  *
  * By using this API, the user agrees to the terms and conditions as stated in the
  * document "Resonon API Terms of Use", located on the Resonon website
  * at: http://www.resonon.com/downloads/Resonon_API_Terms_of_Use.pdf.
  *
  */
 
 
 #ifndef __GUARD_RESONON_IMAGER_BASLER_H
 #define __GUARD_RESONON_IMAGER_BASLER_H
 #include "resonon_imager_base.h"
 #include <stdexcept>
 
 namespace Resonon
 {
     class BaslerCameraImplementation;
 
     class RESONONDLL PikaBasler : public ResononImagerBase
     {
         public:
             PikaBasler();
             virtual ~PikaBasler();
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
             void set_spectral_bin(int new_spectral_bin);
             int get_spectral_bin();
             int get_min_spectral_bin();
             int get_max_spectral_bin();
             int get_band_count();
             int get_start_band();
             void set_start_band(int band);
             int get_min_start_band();
             int get_max_start_band();
             int get_inc_start_band();
             int get_end_band();
             void set_end_band(int band);
             int get_min_end_band();
             int get_max_end_band();
             int get_inc_end_band();
             int get_sample_count();
             int get_start_sample();
             void set_start_sample(int sample);
             int get_min_start_sample();
             int get_max_start_sample();
             int get_inc_start_sample();
             int get_end_sample();
             void set_end_sample(int sample);
             int get_min_end_sample();
             int get_max_end_sample();
             int get_inc_end_sample();
             void set_framerate(const double frames_per_second);
             double get_framerate();
             double get_min_framerate();
             double get_max_framerate();
             double get_min_integration_time();
             double get_max_integration_time();
             void set_integration_time(const double milliseconds);
             double get_integration_time();
             void set_gain(const double gain);
             double get_gain();
             double get_min_gain();
             double get_max_gain();
             void set_internal_trigger();
             void set_external_trigger(unsigned int signal_line, bool rising_edge=true);
             bool is_trigger_external();
         protected:
             BaslerCameraImplementation * pimpl;
     };
 
 } // end namespace Resonon
 #endif //end ifndef __GUARD_RESONON_IMAGER_BASLER_H
 
