#include "speeduino_data.h"

const char ENGINE_STATUS_CHAR[] = {'R', 'C', 'A', 'W', 'a', 'd', '<', '>'};
const char ADVANCE_FORMAT[] = {'%', '3', 'd', char(223), ' ', '\0'};

const int TEMPERATURE_OFFSET = 40;
const unsigned long WAITING_INTERVAL = 100UL;
const unsigned long POLLING_INTERVAL = 1000UL;
