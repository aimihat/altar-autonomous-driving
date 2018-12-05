#pragma once

#include "AltarStateMachine.h"
#include "boost/sml.hpp"
namespace sml = boost::sml;


namespace {
/*! Event struct */
struct start {
    tInt16 id;
};

struct stop {
};

struct startFromPark{

};

struct followLane {
};

struct eBrakeOn{

};

struct onRamp {};

struct eBrakeOff{};

struct childAhead{};

struct junction {
    bool dir;
};

struct exitJunction {};
struct backing {};
struct pullOut {
    bool dir;
};
struct endParking{};

struct brakeLightOn{};
struct brakeLightOff{};
struct hazardLightOn{};
struct hazardLightOff{};
struct reverseLightOn{};
struct reverseLightOff{};
struct leftLightOn{};
struct leftLightOff{};
struct rightLightOn{};
struct rightLightOff{};

struct parkingIn{};
struct parked{};

struct complete{};
struct completed{};



//
//    /*! State */

// termination implemented by X
constexpr const static auto idle = sml::state<class idle>; /*! "idle"_s */
constexpr const static auto ramping = sml::state<class ramping>;
constexpr const static auto merging = sml::state<class merging>;
constexpr const static auto follow = sml::state<class follow>;
constexpr const static auto emergency = sml::state<class emergency>; /*! any case where unexpected things happened */
constexpr const static auto pullOff = sml::state<class pullOff>; /*! emergency vehicle */
constexpr const static auto obstacle = sml::state<class obstacle>;
constexpr const static auto park = sml::state<class park>;
constexpr const static auto parkedIn = sml::state<class parkedIn>;
constexpr const static auto crossing = sml::state<class crossing>;
constexpr const static auto overtaking = sml::state<class overtaking>;

/*! Flags */
bool f_onRamp = false;
bool eBrakeApplied = false;

/* Guard */
auto rampGuard = [] () {return !(f_onRamp);};

/*! Actions */
auto flagRamp = [] () {f_onRamp = !(f_onRamp);};
auto forceStop = [] (cAltarStateMachine& parent_) {LOG_INFO("Jury signalled Stop, stopping...");
    parent_.TransmitLight(HEAD_OFF); eBrakeApplied = true; parent_.requestEBrake();};

auto useLane = [] (cAltarStateMachine& parent_) {parent_.useLaneFollow();};
auto useMapPath = [] (cAltarStateMachine& parent_) {parent_.useMap();};
auto junctionLight = [](const auto &evt) {
    if (evt.dir) {
        sml::process(rightLightOn{});
    } else {
        sml::process(leftLightOn{});
    }
};
auto turnLightsOff = []() {
    sml::process(leftLightOff{});
    sml::process(rightLightOff{});
};

auto usePark = [] (cAltarStateMachine& parent_) {parent_.useParkingSS();};
auto useParkPath = [] (cAltarStateMachine& parent_) {parent_.useParkingPath();};
auto pullOutFromPark = [] (const auto& evt, cAltarStateMachine& parent_) {junctionLight(evt); parent_.useParkingSS();};

/*! State Machines */
struct runImpl {
    auto operator()() const noexcept {
        using namespace sml;
        return make_transition_table(
                // Remember that on entry maneuver will be updated!!
                * "idle"_s + sml::on_entry<_> / [] (cAltarStateMachine &parent_) {parent_.RequestManeuver();},
                "idle"_s + event<followLane> / useLane,
                "idle"_s + event<onRamp> = ramping,
                "idle"_s + event<junction> / junctionLight = "use_map"_s,
                "use_map"_s + sml::on_entry<_> / useMapPath,
                "use_map"_s + event<junction> / useMapPath,
                "use_map"_s + event<exitJunction> / turnLightsOff = "idle"_s,
                /*! Ramping */
                ramping + sml::on_entry<_> / flagRamp,

                /*! Parking */
                "idle"_s + event<parkingIn> / process(leftLightOn{}) = park,
                park + sml::on_entry<_> / usePark,
                park + event<parkingIn> / usePark,
                park + event<backing> / process(leftLightOff{}) = "parking_in"_s,
                "parking_in"_s + sml::on_entry<_> / useParkPath,
                "parking_in"_s + event<backing> = "parking_in"_s,
                "parking_in"_s + event<parked> / process(eBrakeOn{}) = parkedIn,
                "idle"_s + event<startFromPark> = parkedIn,
                parkedIn + sml::on_entry<_> / process(hazardLightOn{}),
                parkedIn + sml::on_exit<_> / [] () { process(hazardLightOff{}); process(eBrakeOff{});},
                parkedIn + event<pullOut> / pullOutFromPark = "pulling_out"_s,
                "pulling_out"_s + event<pullOut> / pullOutFromPark = "pulling_out"_s,
                "pulling_out"_s + event<endParking> / turnLightsOff = "idle"_s,

                /* emergency */
                * "emergency_off"_s + sml::on_entry<_> / [] () {LOG_DUMP("Entered substate");},
                "emergency_off"_s + event<eBrakeOn> [rampGuard] = "emergency_on"_s,
                "emergency_on"_s + sml::on_entry<_> / [] (cAltarStateMachine &parent_) {parent_.requestEBrake(); eBrakeApplied = true;
                    process(brakeLightOn{});},
                    "emergency_on"_s + event<eBrakeOn> = "emergency_on"_s,
                "emergency_on"_s + event<eBrakeOff> / [] () {eBrakeApplied = false;} = "emergency_off"_s,
                "emergency_off"_s + sml::on_entry<_> / [] () {eBrakeApplied = false; process(brakeLightOff{});},

                /* Lights */
                * "brake_off"_s + event<brakeLightOn> = "brake_on"_s,
                "brake_off"_s + sml::on_entry<_> / [] (cAltarStateMachine &parent_) { parent_.TransmitLight(BRAKE_OFF);},
                "brake_on"_s + sml::on_entry<_> / [] (cAltarStateMachine &parent_) { parent_.TransmitLight(BRAKE_ON);},
                "brake_on"_s + event<brakeLightOff> = "brake_off"_s,

                "hazard_on"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(HAZARD_ON);},
                *"hazard_off"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) {
                    parent_.TransmitLight(HAZARD_OFF);},
                "hazard_off"_s + event<hazardLightOn> = "hazard_on"_s,
                "hazard_on"_s + event<hazardLightOff> = "hazard_off"_s,

                "reverse_on"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(REVERSE_ON);},
                *"reverse_off"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) {
                    parent_.TransmitLight(REVERSE_OFF);},
                "reverse_off"_s + event<reverseLightOn> = "reverse_on"_s,
                "reverse_on"_s + event<reverseLightOff> = "reverse_off"_s,

                "left_on"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(LEFT_ON);},
                *"left_off"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(LEFT_OFF);},
                "left_off"_s + event<leftLightOn> = "left_on"_s,
                "left_on"_s + event<leftLightOff> = "left_off"_s,

                "right_on"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(RIGHT_ON);},
                *"right_off"_s + sml::on_entry<_> / [] (cAltarStateMachine& parent_) { parent_.TransmitLight(RIGHT_OFF);},
                "right_off"_s + event<rightLightOn> = "right_on"_s,
                "right_on"_s + event<rightLightOff> = "right_off"_s

        );
    };
};

auto running = sml::state<runImpl>;
//auto running = sml::state<class running>;


struct decisionMakerImpl {

    auto operator()() const noexcept {
        using namespace sml;
        return make_transition_table(
                *idle + sml::on_entry<_> / [](cAltarStateMachine &parent_){/*parent_.TransmitLight(HEAD_OFF, parent_.getClock()->GetTime());*/},

                /* normal loop */
                idle + event<start> / [] (const auto &event, cAltarStateMachine &parent_) {
                    LOG_INFO("Start signal received, starting at %d", event.id);
                    parent_.TransmitLight(HEAD_ON);
                    eBrakeApplied = false;
                    return;
                } = running,

                running + event<stop> / forceStop = idle,
                running + event<completed> / [] (cAltarStateMachine& parent_) {LOG_INFO("Track completed");
                    parent_.TransmitLight(HEAD_OFF); parent_.OutputSpeed(0);} = idle

        );
    };
};

struct my_logger {
    template<class SM, class TEvent>
    void log_process_event(const TEvent &) {
        printf("[%s][process_event] %s\n", sml::aux::get_type_name<SM>(), sml::aux::get_type_name<TEvent>());
    }

    template<class SM, class TGuard, class TEvent>
    void log_guard(const TGuard &, const TEvent &, bool result) {
        printf("[%s][guard] %s %s %s\n", sml::aux::get_type_name<SM>(), sml::aux::get_type_name<TGuard>(),
               sml::aux::get_type_name<TEvent>(), (result ? "[OK]" : "[Reject]"));
    }

    template<class SM, class TAction, class TEvent>
    void log_action(const TAction &, const TEvent &) {
        printf("[%s][action] %s %s\n", sml::aux::get_type_name<SM>(), sml::aux::get_type_name<TAction>(),
               sml::aux::get_type_name<TEvent>());
    }

    template<class SM, class TSrcState, class TDstState>
    void log_state_change(const TSrcState &src, const TDstState &dst) {
        printf("[%s][transition] %s -> %s\n", sml::aux::get_type_name<SM>(), src.c_str(), dst.c_str());
    }
};

//my_logger logger;

}
