#!/usr/bin/env bash
# Particle Tachyon leverages the Qualcomm QCM6490 chipset, which includes integrated GNSS (GPS)
#
capture_location_info() {
    while [ 1 ]; do
        gnss=$(./build/ctl/particle-tachyon-ril-ctl gnss)
        lat=$(echo "$gnss" | grep latitude)
        lon=$(echo "$gnss" | grep longitude)
        speed=$(echo "$gnss" | grep speed)
        last_lock=$(echo "$gnss" | grep last_lock_time_ms)
		fix=$(echo "$gnss" | grep gpssta)
        altit=$(echo "$gnss" | grep altitude)	
        dat_tim=$(echo "$gnss" | grep utc)
        echo "$(date): $lat, $lon, $speed, $last_lock $fix $altit $dat_tim" | tee -a speed.log
        sleep 1
    done
}

capture_location_info