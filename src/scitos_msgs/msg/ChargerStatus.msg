# The status flags of the charger
Header header
bool charging 
bool empty 
bool full 
bool derating #Bit 3: 1=derating, 0=not derating (unused)
bool charging_disabled # (only intern)
bool const_volt_mode  
bool const_current_mode 
bool internal_error_flag 

