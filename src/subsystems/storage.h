#include <Preferences.h>

// saves settings on device between power cycles
class Storage {
public:
  void init();
  int get_target_rpm();
  void set_target_rpm(int rpm);
  float get_accel_correction(int rpm);
  void set_accel_correction(int rpm, float corr);
  int get_trans_trim();
  void set_trans_trim(int idx);

private:
  Preferences preferences;
};

Storage *get_active_store();
