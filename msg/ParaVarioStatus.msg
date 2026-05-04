# Paraglider variometer status
#
# Total-energy compensated vertical speed and derived flight indicators
# computed by the para_vario module.

uint64 timestamp                # [us] time since system start

float32 vario_raw               # [m/s] uncompensated climb rate (-vz NED), positive up
float32 vario_tek               # [m/s] total-energy-compensated vario, positive up
float32 airspeed_tas            # [m/s] true airspeed used in TEK term, NaN if invalid
float32 airspeed_deriv          # [m/s^2] filtered airspeed derivative used in TEK term
float32 ground_speed            # [m/s] horizontal ground speed
float32 glide_ratio             # [-] ground_speed / sink_rate, 0 when not sinking
float32 baro_alt                # [m] barometric altitude (MSL, ISA-corrected)

uint8 STATE_IDLE   = 0
uint8 STATE_CLIMB  = 1
uint8 STATE_SINK   = 2
uint8 STATE_GLIDE  = 3
uint8 state                     # discrete vario state

bool airspeed_valid             # true if TEK term used real airspeed
bool igc_logging                # true if IGC writer is appending B-records
