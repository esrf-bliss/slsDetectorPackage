
#include "catch.hpp"

#include "ClientSocket.h"
#include "logger.h"
#include "slsDetector.h"
#include "sls_detector_defs.h"

#include "Timer.h"
#include "sls_detector_funcs.h"
#include <iostream>
#include <vector>
#define VERBOSE

//Header holding all configurations for different detectors
#include "config.h"

TEST_CASE("single EIGER detector no receiver basic set and get", "[.integration]") {
    //TODO! this test should take command line arguments for config
    SingleDetectorConfig c;

    //Read type by connecting to the detector
    auto type = slsDetector::getTypeFromDetector(c.hostname);
    CHECK(type == c.type_enum);

    //Create slsDetector of said type and set hostname and detector online
    slsDetector d(type);
    CHECK(d.getDetectorTypeAsEnum() == type);
    CHECK(d.getDetectorTypeAsString() == c.type_string);

    d.setHostname(c.hostname);
    CHECK(d.getHostname() == c.hostname);

    d.setOnline(true);
    CHECK(d.getOnlineFlag() == true);

    CHECK(d.getReceiverOnline() == false);
    CHECK(d.checkDetectorVersionCompatibility() == slsDetectorDefs::OK);

    //Setting and reading exposure time
    auto t = 1000000000;
    d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME, t);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME) == t);

    //size of an eiger half module with and without gap pixels
    CHECK(d.getTotalNumberOfChannels() == 256 * 256 * 4);
    CHECK(d.getTotalNumberOfChannels(slsDetectorDefs::dimension::X) == 1024);
    CHECK(d.getTotalNumberOfChannels(slsDetectorDefs::dimension::Y) == 256);
    // CHECK(d.getTotalNumberOfChannels(slsDetectorDefs::dimension::Z) == 1);
    CHECK(d.getTotalNumberOfChannelsInclGapPixels(slsDetectorDefs::dimension::X) == 1024);
    CHECK(d.getTotalNumberOfChannelsInclGapPixels(slsDetectorDefs::dimension::Y) == 256);
    // CHECK(d.getTotalNumberOfChannelsInclGapPixels(slsDetectorDefs::dimension::Z) == 1);

    CHECK(d.getNChans() == 256 * 256);
    CHECK(d.getNChans(slsDetectorDefs::dimension::X) == 256);
    CHECK(d.getNChans(slsDetectorDefs::dimension::Y) == 256);
    // CHECK(d.getNChans(slsDetectorDefs::dimension::Z) == 1);

    CHECK(d.getNChips() == 4);
    CHECK(d.getNChips(slsDetectorDefs::dimension::X) == 4);
    CHECK(d.getNChips(slsDetectorDefs::dimension::Y) == 1);
    // CHECK(d.getNChips(slsDetectorDefs::dimension::Z) == 1);

    d.freeSharedMemory();
}

TEST_CASE("Set control port then create a new object with this control port",
          "[.integration]") {
    /*
    TODO!
    Standard port but should not be hardcoded 
    Is this the best way to initialize the detectors
    Using braces to make the object go out of scope
    */
    int old_cport = DEFAULT_PORTNO;
    int old_sport = DEFAULT_PORTNO + 1;
    int new_cport = 1993;
    int new_sport = 2000;
    SingleDetectorConfig c;
    {
        auto type = slsDetector::getTypeFromDetector(c.hostname);
        CHECK(type == c.type_enum);
        slsDetector d(type);
        d.setHostname(c.hostname);
        d.setOnline(true);
        CHECK(d.getControlPort() == old_cport);
        d.setControlPort(new_cport);
        CHECK(d.getStopPort() == old_sport);
        d.setStopPort(new_sport);
        d.freeSharedMemory();
    }
    {
        auto type = slsDetector::getTypeFromDetector(c.hostname, new_cport);
        CHECK(type == c.type_enum);
        slsDetector d(type);
        d.setHostname(c.hostname);
        d.setControlPort(new_cport);
        d.setStopPort(new_sport);
        CHECK(d.getControlPort() == new_cport);
        CHECK(d.getStopPort() == new_sport);

        d.setOnline(true);

        //Reset standard ports
        d.setControlPort(old_cport);
        d.setStopPort(old_sport);
        d.freeSharedMemory();
    }

    auto type = slsDetector::getTypeFromDetector(c.hostname);
    CHECK(type == c.type_enum);
    slsDetector d(type);
    d.setHostname(c.hostname);
    d.setOnline(true);
    CHECK(d.getStopPort() == DEFAULT_PORTNO + 1);
    d.freeSharedMemory();
}

TEST_CASE("Locking mechanism and last ip", "[.integration]") {
    SingleDetectorConfig c;
    auto type = slsDetector::getTypeFromDetector(c.hostname);
    slsDetector d(type);
    d.setHostname(c.hostname);
    d.setOnline(true);

    //Check that detector server is unlocked then lock
    CHECK(d.lockServer() == 0);
    d.lockServer(1);
    CHECK(d.lockServer() == 1);

    //Can we still access the detector while it's locked
    auto t = 1300000000;
    d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME, t);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME) == t);

    //unlock again and free
    d.lockServer(0);
    CHECK(d.lockServer() == 0);

    CHECK(d.getLastClientIP() == c.my_ip);
    d.freeSharedMemory();
}

TEST_CASE("Excersise all possible set timer functions", "[.integration]") {
    // FRAME_NUMBER, /**< number of real time frames: total number of acquisitions is number or frames*number of cycles */
    // ACQUISITION_TIME, /**< exposure time */
    // FRAME_PERIOD, /**< period between exposures */
    // DELAY_AFTER_TRIGGER, /**< delay between trigger and start of exposure or readout (in triggered mode) */
    // GATES_NUMBER, /**< number of gates per frame (in gated mode) */
    // CYCLES_NUMBER, /**< number of cycles: total number of acquisitions is number or frames*number of cycles */
    // ACTUAL_TIME, /**< Actual time of the detector's internal timer */
    // MEASUREMENT_TIME,  /**< Time of the measurement from the detector (fifo) */

    // PROGRESS, /**< fraction of measurement elapsed - only get! */
    // MEASUREMENTS_NUMBER,
    // FRAMES_FROM_START,
    // FRAMES_FROM_START_PG,
    // SAMPLES,
    // SUBFRAME_ACQUISITION_TIME, /**< subframe exposure time */
    // STORAGE_CELL_NUMBER, /**<number of storage cells */
    // SUBFRAME_DEADTIME, /**< subframe deadtime */
    // MEASURED_PERIOD,	/**< measured period */
    // MEASURED_SUBPERIOD,	/**< measured subperiod */
    // MAX_TIMERS
    SingleDetectorConfig c;
    auto type = slsDetector::getTypeFromDetector(c.hostname);
    slsDetector d(type);
    d.setHostname(c.hostname);
    d.setOnline(true);

    //Number of frames
    auto frames = 10;
    d.setTimer(slsDetectorDefs::timerIndex::FRAME_NUMBER, frames);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::FRAME_NUMBER) == frames);

    auto t = 10000000;
    d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME, t);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME) == t);

    auto period = 1000000000;
    d.setTimer(slsDetectorDefs::timerIndex::FRAME_PERIOD, period);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::FRAME_PERIOD) == period);

    // not implemented for EIGER
    // auto delay = 10000;
    // d.setTimer(slsDetectorDefs::timerIndex::DELAY_AFTER_TRIGGER, delay);
    // CHECK(d.setTimer(slsDetectorDefs::timerIndex::DELAY_AFTER_TRIGGER) == delay);

    // auto gates = 1;
    // d.setTimer(slsDetectorDefs::timerIndex::GATES_NUMBER, gates);
    // CHECK(d.setTimer(slsDetectorDefs::timerIndex::GATES_NUMBER) == gates);

    auto cycles = 2;
    d.setTimer(slsDetectorDefs::timerIndex::CYCLES_NUMBER, cycles);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::CYCLES_NUMBER) == cycles);

    auto subtime = 200;
    d.setTimer(slsDetectorDefs::timerIndex::SUBFRAME_ACQUISITION_TIME, subtime);
    CHECK(d.setTimer(slsDetectorDefs::timerIndex::SUBFRAME_ACQUISITION_TIME) == subtime);

    d.freeSharedMemory();
}

// TEST_CASE("Aquire", "[.integration][eiger]"){
//     SingleDetectorConfig c;
//     auto type = slsDetector::getTypeFromDetector(c.hostname);
//     slsDetector d(type);
//     d.setHostname(c.hostname);
//     d.setOnline(true);

//     auto period = 1000000000;
//     auto exptime = 100000000;
//     d.setTimer(slsDetectorDefs::timerIndex::FRAME_NUMBER, 5);
//     d.setTimer(slsDetectorDefs::timerIndex::ACQUISITION_TIME, exptime);
//     d.setTimer(slsDetectorDefs::timerIndex::FRAME_PERIOD, period);
//     d.startAndReadAll();

    

//     auto rperiod = d.getTimeLeft(slsDetectorDefs::timerIndex::MEASURED_PERIOD);
//     CHECK(rperiod == 0.1);

//     d.freeSharedMemory();
// }