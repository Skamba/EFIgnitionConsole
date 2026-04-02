#ifndef SPEEDUINO_DATA_H
#define SPEEDUINO_DATA_H

// Position numbers in Speeduino's real-time data block.
#define SQUIRT               1
#define ENGINE               2
#define DWELL                3
#define MAP_LB               4
#define MAP_HB               5
#define IAT_PLUS_OFFSET      6
#define COOLANT_PLUS_OFFSET  7
#define BAT_CORRECTION       8
#define BATTERY10            9
#define OXIGEN              10
#define EGO_CORRECTION      11
#define IAT_CORRECTION      12
#define WUE_CORRECTION      13
#define RPM_LB              14
#define RPM_HB              15
#define TAE_AMOUNT          16
#define CORRECTIONS         17
#define VE                  18
#define AFR_TARGET          19
#define PW1_LB              20
#define PW1_HB              21
#define TPS_DOT             22
#define ADVANCE_ANGLE       23
#define TPS                 24
#define LOOPS_PER_SECOND_LB 25
#define LOOPS_PER_SECOND_HB 26
#define FREE_RAM_LB         27
#define FREE_RAM_HB         28
#define BOOST_TARGET        29
#define BOOST_DUTY          30
#define SPARK               31
#define RPM_DOT_LB          32
#define RPM_DOT_HB          33
#define ETHANOL_PCT         34
#define FLEX_CORRECTION     35
#define FLEX_IGN_CORRECTION 36
#define IDLE_LOAD           37
#define TEST_OUTPUTS        38
#define OXIGEN2             39
#define BARO                40
#define FUEL_PRESSURE      103

#define BIT_ENGINE_RUN      0
#define BIT_ENGINE_CRANK    1
#define BIT_ENGINE_ASE      2
#define BIT_ENGINE_WARMUP   3
#define BIT_ENGINE_ACC      4
#define BIT_ENGINE_DCC      5
#define BIT_ENGINE_MAPACC   6
#define BIT_ENGINE_MAPDCC   7

extern const char ENGINE_STATUS_CHAR[];
extern const char ADVANCE_FORMAT[];
extern const int TEMPERATURE_OFFSET;
extern const unsigned long WAITING_INTERVAL;
extern const unsigned long POLLING_INTERVAL;

#endif
