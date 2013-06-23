# A message for holding the current battery state. 
# The battery state is described by the current voltage, the battery current, the charge state
# in percent, an estimated life time in minutes and a flag, which indicates if the battery is
# currently charged or not.
#
Header header
float32 voltage            # The battery voltage in [V].
float32 current            # The battery current in [A].
int8 lifePercent           # The battery life percent [0..100] or 255 if unknown.
int16 lifeTime             # The battery life time in minutes or -1 if unknown.
bool charging              # Is the battery currently being charged.
bool powerSupplyPresent    # Is the robot powered by an external power supply.
float32[] cellVoltage      # The voltage of the single battery cells in [V].
