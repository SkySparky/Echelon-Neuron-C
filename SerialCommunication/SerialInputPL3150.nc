//  Echelon Mini Kit, MGDemo Neuron C Example Application
//  Revision 3
//
//  This example demonstrates the implementation of a comprehensive,
//  interoperable, self-installed application. A wide range of features
//  are demonstrated, including manual and automatic enrollment, simple
//  and compound connections including manufacturer-specific compound
//  connections. Due to the comprehensive nature of this example
//  application, it will not fit the limited space of a cost-effective
//  FT 3120 or PL 3120 Smart Transceiver single-chip solution.
//  Suitable target hardware contains most Neuron 3150 based solutions,
//  and Neuron 3120 solutions with 8 kB of onchip EEPROM.
//  See product documentation for more details.
//
//  This application must be linked with the IsiFull.lib library.
//
// Copyright (c) 2005-2009 Echelon Corporation.  All rights reserved.
//
// ECHELON MAKES NO REPRESENTATION, WARRANTY, OR CONDITION OF
// ANY KIND, EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE OR IN
// ANY COMMUNICATION WITH YOU, INCLUDING, BUT NOT LIMITED TO,
// ANY IMPLIED WARRANTIES OF MERCHANTABILITY, SATISFACTORY
// QUALITY, FITNESS FOR ANY PARTICULAR PURPOSE,
// NONINFRINGEMENT, AND THEIR EQUIVALENTS.
//
// Revision 3:
// a)   added IsiPreStart() call to reset task
// b)   correctly initialized some local variables in camOperation()
// c)   added support for optimization and diagnostics control directives introduced
//      with NodeBuilder FX and Mini FX
// d)   added application-specific ISI connection table for improved linkage
//
// Revision 2:
// a)   supress turnaround connections for switch/light pairs when self-installed;
//      see IsiGetAssembly() for more details
// b)   clear usage field in CSMODATA to allow connecting with devices for all usage
//      types, in particular residential use (typical powerline use-case) and industrial
//      or commercial use (typical TP/FT use-case)
// c)   review of program IDs. Program IDs to use with this example application are
//      as follows:
//      PL 3150 Evaluation Board (CENELEC off): 9F.FF.FF.05.01.05.11.04
//      PL 3150 Evaluation Board (CENELEC on):  9F.FF.FF.05.01.05.10.04
//      FT 3150 Evaluation Board:               9F.FF.FF.05.01.04.04.04
// d)   Added support for network variable heartbeats in self-installed mode 
//      See IsiQueryHeartbeat() for more.
// e)   Added support for the CENELEC Configuration Library (disabled by default)
//
//  floating-point support:
//  insert a comment leading the next line to disable definition of the INCLUDE_FLOAT
//  macro. This will remove the optional floating-point temperature output
//  network variable, and reduce the application's memory footprint as the floating-point 
//	library is no longer required. 
#define INCLUDE_FLOAT

// enable definition of the SUPPORT_CCL macro to enable support for the 
// CENELEC Configuration Library (CCL):
#define SUPPORT_CCL

#include <mem.h>
#include <stdlib.h>
#include <string.h>
#include <control.h>
#include <isi.h>

#ifdef  SUPPORT_CCL
#include <cenelec.h>
#endif  //  SUPPORT_CCL

// Define the number of alias table entries.
// For the most compact implementation, no alias table entries are required (set
// value to 0). To support versatile network variable connections when used in a
// managed network, specify a non-zero number of alias table entries.
// This example application follows the rule-of-thumb recommendation provided by
// LonMark International: NumAliases = (NVcount==0) ? 0 : min(62, 10+(NVcount/3))
// Ignoring the Node Object network variables, which typically remain unbound,
// this results in a recommended alias count of 10 + 20/3 = 16
#pragma num_alias_table_entries 16

// Define the number of address table entries.
// One address table entry is sufficient for this application when used exclusively
// with the Mini Example suite, and in a self-installed environment. To support
// versatile connections in the general case, define a larger number of address table
// entries.
// The recommended formula for the minimum number of address table entries is
// NumAddress = min(15, NumAliases + NVcount)
// In this example application, the recommended address table size is 15, therefore:
#pragma num_addr_table_entries  15

// several miscellaneous compiler directives:
#pragma enable_io_pullups

#ifdef _NEURONC
   // The _NEURONC symbol is predefined, starting with NodeBuilder FX and Mini FX.
   // We use this symbols's presence to enable use of enhanced optimizer and diagnostics
   // control directives. 
#   pragma optimization all
#   pragma disable_warning 24   // not enough address table entries for optimium efficiency
#else
#   pragma codegen optimization_on
#   pragma codegen cp_family_space_optimization
#endif  // _NEURONC

#pragma codegen put_cp_template_file_offchip
#pragma codegen put_cp_value_files_offchip
#pragma set_node_sd_string  "MGDemo"

//
//  Declaration of the interoperable interface
//
//

//  The implementation of the SFTPnodeObject functional profile:

//  (a) The file directory
//  For defaults, see the LonMark File Transfer Protocol specification and
//  the LonMark Application Layer Interoperability Guidelines for more details.
//  Both documents can be found at http://www.lonmark.org
const struct {
    int Version;
    int NumberOfFiles;
    struct {
        unsigned long FileSize;
        unsigned long FileType;
        union {
            void* const DataConst;
            void* DataRW;
        }         FileData;
    }   Files[2];
} FileDirectory = {
    0x20,       //  directory version
    2,          //  number of files: template file, and writable value file
    {
        { cp_template_file_len,         2 /* template file */,  cp_template_file},
        { cp_modifiable_value_file_len, 1 /* value file    */,  cp_modifiable_value_file},
    }
};

//  (b) The network variables:
#include <snvt_cfg.h>
#include <snvt_rq.h>
#include <s32.h>
network output sync         SNVT_obj_status nvoStatus; // node object's status output
network output polled const SNVT_address nvoDirectory = (SNVT_address)&FileDirectory;
network input               SNVT_obj_request nviRequest;     // node object's "command" input

//  (c) The configuration properties - note that SCPTnwrkCnfg must be implemented as
//		a configuration network variable to ease transitions between self-installed 
//		and managed networks. Notice this property defaults to CFG_EXTERNAL. This is 
//		the default value recommended by the LonMark Interoperability Guidelines, 
//		and is the best choice when using the device in a managed network. See the 
//		when(reset) task, below, for more details about application start-up.
network input SCPTnwrkCnfg cp cp_info(reset_required) nciNetConfig = CFG_EXTERNAL;
SCPTlocation cp_family cpLocation = {"My Workbench"};

//  (d) The implementation of SFPTnodeObject
fblock SFPTnodeObject {
    // associate network variables implemented in this device with
    // member network variables defined in the functional profile:
    nvoStatus implements nvoStatus;
    nviRequest implements nviRequest;
    nvoDirectory implements nvoFileDirectory;
} fbNodeObject external_name("Node") fb_properties {
    // associate configuration properties with the node object:
    nciNetConfig,
    cpLocation
};

//
//  The implementation of the SFPTclosedLoopActuator and
//  SFPTopenLoopActuator functional profiles
//
#define CHANNELS 4  //  number of actuators and sensors (light/switch pairs)

SCPTmaxSendTime cp_family cpHeartbeatInterval;

//  Notice the declaration of network variable arrays requires the use
//  of bind_info(expand_array_info), unless IsiGet[Next]Assembly() forwarders
//  are overridden with application-specific implementations.
network input  SNVT_switch nviLight[CHANNELS];      // lamp primary (input) NV
network output bind_info(unackd) SNVT_switch nvoLightFb[CHANNELS]; // lamp feedback

fblock SFPTclosedLoopActuator {
    // associate network variables implemented in this device with
    // member network variables defined in the functional profile:
    nviLight[0] implements nviValue;
    nvoLightFb[0] implements nvoValueFb;
} fbLight[CHANNELS] external_name("Light") fb_properties {
    // an implementation-specific CP, added to SFPTclosedLoopActuator
    // Notice the use of the 'static' keyword causes a single heartbeat
    // interval preference to be shared among all functional blocks in
    // this fblock array - each functional block employs an independent
    // heartbeat timer, but all functional blocks in this array share
    // the heartbeat interval.
    static  cpHeartbeatInterval
};

SCPTdefOutput cp_family cpSwitchDefault;

// switch primary (output) NV:
//  Notice the declaration of network variable arrays requires the use
//  of bind_info(expand_array_info), unless IsiGet[Next]Assembly() forwarders
//  are overridden with application-specific implementations.
network output bind_info(unackd_rpt) SNVT_switch nvoSwitch[CHANNELS] nv_properties {
    cpSwitchDefault = { /* value */ 0, /* state */ 0}
};

// switch feedback (input) NV:
network input  SNVT_switch nviSwitchFb[CHANNELS];

fblock SFPTclosedLoopSensor {
    // associate network variables implemented in this device with
    // member network variables defined in the functional profile:
    nvoSwitch[0] implements nvoValue;
    nviSwitchFb[0] implements nviValueFb;
} fbSwitch[CHANNELS] external_name("Switch") fb_properties {
    // an implementation-specific CP, added to SFPTclosedLoopActuator
    // Notice the use of the 'static' keyword causes a single heartbeat
    // interval preference to be shared among all functional blocks in
    // this fblock array - each functional block employs an independent
    // heartbeat timer, but all functional blocks in this array share
    // the heartbeat interval.
    static  cpHeartbeatInterval
};


//
//  SFPThvacTempSensor implementation
//
//  Throttle and minimum delta configuration properties. See the
//  ProcessThermometer function, below, for discussion and use of these
//  parameters.
SCPTminSendTime cp_family cpThrottle;
SCPTminDeltaTemp cp_family cpDelta;

network output bind_info(unackd_rpt) SNVT_temp_p nvoTemperature nv_properties {
    cpDelta = 50    // 0.5 degrees Celsius minimum between two values
};

#ifdef INCLUDE_FLOAT
// The SFPThvacTempSensor defines an optional output network variable
// member that provides the temperature reading as floating point
// data. The INCLUDE_FLOAT macro controls whether this network variable
// will be implemented, and whether the related floating-point libraries
// will be linked with the application.
#   include <float.h>
network output bind_info(unackd_rpt) SNVT_temp_f nvoTemperatureF;
#endif  // include_float

fblock SFPThvacTempSensor {
    nvoTemperature implements nvoHVACTemp;
#ifdef  INCLUDE_FLOAT
    nvoTemperatureF implements nvoFloatTemp;
#endif
} fbThermometer external_name ("Thermometer") fb_properties {
    cpHeartbeatInterval,
    cpThrottle
};

//
//  SFPTopenLoopActuator (buzzer) implementation
//
network input SNVT_freq_hz nviFrequency = 0;    
network input SNVT_switch  nviEnable;

fblock SFPTopenLoopActuator {
    nviFrequency implements nviValue;
    // Note that this implementation adds an implementation_specific
    // network variable to the standard functional profile. This extension
    // provides added functionality to the functional block. A user-defined
    // functional block is not required, as no aspect of the standard
    // functional block has been removed or changed in any way.
    //
    // Note this implementation-specific network variable enables or disables
    // the buzzer. The alternative approach is to update the buzzer's input
    // network variable with 0 (zero); the improvement with this extension is
    // to support preserving the most recent frequency value.
    // Note that disabling the entire functional block with RQ_DISABLED
    // commands to the Node Object has a different effect: a disabled functional
    // block shall not change it's network or physical outputs.
    nviEnable implementation_specific(512) nviEnable;
} fbBuzzer external_name ("Buzzer");


//
//  The following variable is used for device management duties.
//  Notice the declaration with the 'eeprom' modifier, causing the
//  variable to be persistent. Initial values for these variables are
//  only set at application download, but persist a reset.
//
//  Implementing one persistent state variable for all functional blocks,
//  the N-th bit (starting with N=0 for the LSB) represents the boolean
//  disabled state for fblock N, where N equals the block's global index.
//
#define FBLOCK_DISABLED 0x01u

far offchip eeprom unsigned long Disabled = 0;
//
//  Switch/Light pairs 1 and 2 (i.e. SW1/LED1 + SW2/LED2) have an overloaded
//  behavior: when a network addressing conflict is detected and repaired,
//  LED1 starts flashing and will no longer respond to normal light/switch
//  operational inputs. In this mode, SW1 must be used to return to normal
//  operation.
//  Likewise, LED2 might flash to indicate a network variable selector conflict
//  that was detected and repaired. In this mode, SW2 must be used to return
//  to normal light/switch operation.
//  Although switch/light pairs 3 and 4 have no overloaded behavior, the
//  framework provided is prepared to overload all four assemblies.
far unsigned AlternativeMode[CHANNELS];

//  The following array holds heartbeat timer variables for each of the
//  switches, the lights, and the thermometer
far SCPTmaxSendTime Heartbeat[2*CHANNELS+1];

//
//  Utilities
//
boolean IsEnabled(unsigned FbIndex) {
    return !(boolean)(Disabled & (FBLOCK_DISABLED << FbIndex));
}

//
//  I/O declarations for MiniGizmo buttons
//
//  The buttons are connected to a 74HC165 8-bit parallel-in/serial-out
//  shift register. Data is shifted on IO.4 (clock) and IO.5 (data),
//  with a latch on IO.6 (low-active to capture parallel input).
//  The MG_BUTTONS_DEBOUNCE macro determines how many times the buttons
//  are read for debouncing.
//  See GetButtons() for example use
IO_4 input bitshift numbits(8) clockedge(-) ioButtons;
IO_6 output bit ioButtonLd = 1;
#define MG_BUTTONS_DEBOUNCE 3

unsigned GetButtons(void) {
    unsigned Debounce, Buttons;
    Buttons = 0xFF;
    for (Debounce = 0; Debounce < MG_BUTTONS_DEBOUNCE; ++Debounce) {
        // capture parallel lines:
        io_out(ioButtonLd, 0);
        // deactivate capture. The 74HC165 requires no more than 100ns for
        // capture; we cannot beat this even with a 40MHz Smart Transceiver
        // or Neuron Chip. No need to consider load pulse timing, therefore:
        io_out(ioButtonLd, 1);
        // take a sample and debounce:
        Buttons &= (unsigned)io_in(ioButtons);
    }
    return ~Buttons;
}

//
//  I/O declarations for MiniGizmo LEDs
//
//  The LEDs are connected to a 74HC595 8-bit serial-in/parallel-out shift
//  register. Data is shifted on IO.2 (clock) and IO.3 (data), a rising edge
//  on IO.1 strobes data into the latch.
//  See SetLEDs() for example use.
IO_2 output bitshift numbits(8) ioLEDs;
IO_1 output bit ioLEDLd = 1;

far unsigned PreviousLEDs = 0;

void SetLEDs(unsigned LEDs, unsigned Mask) {
    // We may only want to set some LEDs, indicated by Mask. Bits outside the
    // mask are added from the previous pattern:
    LEDs |= PreviousLEDs & ~Mask;
    PreviousLEDs = LEDs;

    // LEDs are driven active low - the SetLEDs function handles the inversion
    // so that the application developer may think in positive logic:
    io_out(ioLEDs, ~LEDs);
    // strobe:
    io_out(ioLEDLd, 0);
    io_out(ioLEDLd, 1);
}

//
//  I/O declarations for MiniGizmo Piezzo
//
//  The Piezzo buzzer is connected to IO.0, directly driven by the Smart
//  Transceiver or Neuron Chip.
IO_0 output frequency clock(0) invert ioPiezzo = 1;

//
//  I/O declarations for the MiniGizmo temperature Sensor
//
//  The MiniGizmo temperature sensor uses a 1-Wire Dallas DS18S20 digital
//  thermometer device. Since the MiniGizmo uses only one device on the
//  1-Wire bus, this implementation uses a simplified protocol, skipping the
//  search ROM step. See GetTemperature() for example use.
//  (1-Wire is a registered trademark of Dallas Semiconductor)
//  You can find out more about this device on www.maxim-ic.com
IO_7 touch ioThermometer;
#define DS18S20_SKIP_ROM    0xCCu
#define DS18S20_CONVERT     0x44u
#define DS18S20_READ        0xBEu

// The DS18S20 might take a while to convert, and the conversion time is
// a function of the temperature read. Thus, the GetTemperature function
// always provides the value most recently read into the application. If
// no conversion is under way, a new conversion cycle is started.
SNVT_temp_p GetTemperature(void) {
    union {
        SNVT_temp_p snvtTempP;
        unsigned    Bytes[2];
    } CurrentTemperature;
    CurrentTemperature.snvtTempP = 32767l;


    if(touch_reset(ioThermometer)) {
        (void) touch_byte(ioThermometer, DS18S20_SKIP_ROM);
        (void) touch_byte(ioThermometer, DS18S20_READ);

        CurrentTemperature.Bytes[1] = touch_byte(ioThermometer, 0xFFu); // low
        CurrentTemperature.Bytes[0] = touch_byte(ioThermometer, 0xFFu); // high

        if(touch_reset(ioThermometer)) {
            //  The value currently held in TemperatureDataBuffer is the raw DS18S20
            //  data, in Celsius, at a resolution of 0.5 degrees. SNVT_temp_p, however,
            //  provides a resolution of 0.01 in a fixed-point implementation.
            //  We must correct the raw reading by factor 50 thus:
            CurrentTemperature.snvtTempP *= 50l;

            // start the next conversion cycle:
            (void) touch_byte(ioThermometer, DS18S20_SKIP_ROM);
            (void) touch_byte(ioThermometer, DS18S20_CONVERT);
        } else {
            CurrentTemperature.snvtTempP = 32767l;
        }
    }
    return CurrentTemperature.snvtTempP;
}


#ifdef  SUPPORT_CCL
eeprom boolean firstStart = TRUE;

#define	CAM_CONFIRM		0x01	// SW1/LED1 and flag in internal state vector
#define	CAM_CENELEC 	0x80	// SW8/LED8 and flag in internal state vector

// implement cenelec configuration behavior:
void camOperation(void) {
    enum {
        flashOff    = 0,
        flashOn     = CAM_CONFIRM
    } flash;
    boolean isConfirmed;
    unsigned currentButtons, previousButtons, timer;

    flash = timer = 0;
    SetLEDs(isConfirmed = previousButtons = 0x00, 0xFF);

    if(camIsSpmCBand()) {
        firstStart = TRUE;
        camQuietMode();

        do {
            SetLEDs(camIsCenelecEnabled() ? CAM_CENELEC : 0x00, CAM_CENELEC);

            currentButtons = GetButtons();

            if(currentButtons != previousButtons) {
                previousButtons = currentButtons;

                if(currentButtons & CAM_CENELEC) {
                    camSetCenelec(!camIsCenelecEnabled());
                }
                isConfirmed = currentButtons & CAM_CONFIRM;
            }

            // note two warnings will be issued by the following code sequence, related to the 
            // use of the uninitialized variable "timer", and that of "flash". It is true that 
            // these variables are uninitialized, but their initial state is irrelevant for the 
            // operation of this algorithm and we can therefore save a small amount of code space 
            // by choosing not to initialize these locals.
            if(!timer) {
                SetLEDs(flash, CAM_CONFIRM);
                flash ^= CAM_CONFIRM;
            }
            ++timer;

            post_events();
        } while(!isConfirmed);
        firstStart = FALSE;
        node_reset();
    }
}
#endif  //  SUPPORT_CCL

//
// Reset Processing
//
// The when(reset) task executes when the device is configured and completes a
// reset. To control the ISI engine start-up, or to prevent the ISI engine from
// starting-up, reset processing contains the following logic:
// a)	If this is the first reset with a new application image, the reset code
//		sets nciNetConfig to CFG_LOCAL. This allows the ISI engine to start on a 
//		brand new device. The initial value of CFG_NUL of the local, persistent, 
//		OldNwrkCnfg variable is used to detect the first start 
// b) 	If nciNetConfig is set to CFG_LOCAL but the previous value is CFG_EXTERNAL
//		(determined by the tracking variable OldNwrkCnfg), the device returns itself
//		to factory defaults
// c)	If nciNetConfig is set to CFG_LOCAL, the ISI engine starts
far offchip eeprom SCPTnwrkCnfg OldNwrkCnfg = CFG_NUL;

when(reset) {
    // we play a little with the LEDs and the buzzer.
    // This is used to visually detect a healthy
    // node during manufacture and installation:
    SCPTnwrkCnfg cpNwrkConfig;
    unsigned Count;

    // The following call to IsiPreStart() is only effective with target chips containing
    // the ISI core engine in ROM (such as the PL 3170 Smart Transceiver), and it does 
    // nothing otherwise. 
    IsiPreStart();

#ifdef  SUPPORT_CCL
    if(firstStart || service_pin_state()) {
        camOperation();
    }
#endif  //  SUPPORT_CCL

    SetLEDs(0x55u, 0xFFu);  // LEDs into defined state
    io_out(ioPiezzo, 7599ul);

    cpNwrkConfig = OldNwrkCnfg; 

    //  set the default output values for the switches:
    for(Count = 0; Count < CHANNELS; ++Count) {
        fbSwitch[Count]::nvoValue = fbSwitch[Count]::nvoValue::cpSwitchDefault;
    }

    if(cpNwrkConfig == CFG_NUL) {
        // for the first application start, set nciNetConfig to CFG_LOCAL, thus allow the 
        // ISI engine to run by default: 
        fbNodeObject::nciNetConfig = CFG_LOCAL;
    }

    OldNwrkCnfg = fbNodeObject::nciNetConfig;

    if(fbNodeObject::nciNetConfig != CFG_LOCAL) {
        //  load the heartbeat timers:
        for(Count = 0; Count < CHANNELS; ++Count) {
            Heartbeat[Count] = fbLight[Count]::cpHeartbeatInterval;
            Heartbeat[CHANNELS+Count] = fbSwitch[CHANNELS+Count]::cpHeartbeatInterval;
        }
        Heartbeat[2*CHANNELS] = fbThermometer::cpHeartbeatInterval;
    } else {
        //
        //	we are in self-installed mode:
        //
        if(cpNwrkConfig == CFG_EXTERNAL) {
            //	The application has just returned into the self-installed environment. 
            //  Make sure to re-initialize the entire ISI engine:
            IsiReturnToFactoryDefaults();   // Call NEVER returns! (resets the device)
        }
        //  We are in a self-installed network:
        //  Start the ISI engine:
        IsiStartS(isiFlagHeartbeat | isiFlagExtended);
    }

    SetLEDs(0xAAu, 0xFFu);  // LEDs into defined state
    io_out(ioPiezzo, 9579ul);
    delay(1250ul);

    // get the thermometer going:
    (void)GetTemperature();

    // enable the buzzer:
    nviEnable.state = 1;
    nviEnable.value = 200u;

    SetLEDs(0, 0xFFu);
    io_out(ioPiezzo, 0);
}

//
//  Input Processing
//
void ProcessIsiButton(unsigned Assembly, boolean Constructive, boolean Alternative);

#define TICKS_PER_SECOND    50u
#define BUTTON_SHORT    ((3*(1000/TICKS_PER_SECOND))/2) //  1.5 seconds
#define BUTTON_LONG     (5*BUTTON_SHORT)

far unsigned ButtonDuration[CHANNELS];
far unsigned ButtonHistory;

unsigned Next(unsigned Buttons) {
    Buttons &= ~0x11u;
    return Buttons >> 1u;
}

void ProcessButtons(unsigned Buttons) {
    unsigned Assembly, Duration, PreviousButtons, Led;

    PreviousButtons = ButtonHistory;
    ButtonHistory = Buttons;

    Led = 0x01;
    for(Assembly = 0; Assembly < CHANNELS; ++Assembly) {
        Duration = ButtonDuration[Assembly];

        if(!(PreviousButtons & 0x11u) && ((Buttons & 0x11u) == 0x01u)) {
            // SW1..SW4, no SW5..SW8:
            if(AlternativeMode[Assembly]) {
                // cancel alternative mode:
                AlternativeMode[Assembly] = 0;
                // return light to the last known state:
                SetLEDs(nvoLightFb[Assembly].state ? Led : 0, Led);
            } else if(IsEnabled(fbSwitch[Assembly]::global_index)) {
                // toggle the switch position:
                nvoSwitch[Assembly].state ^= 1;
                if(nvoSwitch[Assembly].state) {
                    nvoSwitch[Assembly].value = 200u;
                }
                nviSwitchFb[Assembly] = nvoSwitch[Assembly];

                // With the ISI engine running, the MGDemo example application provides 4 connection 
                // assemblies, each of which is made from a switch/light pair. In this mode, the switch/light
                // pairs cannot be separated and are coupled on an application-level. When the device is used
                // in a managed network, however, 8 distinct and separate functional blocks (4 sensors for 
                // the switches, 4 actuators for the lights) are available as individual units:
                if(IsiIsRunning()) {
                    SetLEDs(nvoSwitch[Assembly].state ? Led : 0, Led);
                    nviLight[Assembly] = nvoLightFb[Assembly] = nvoSwitch[Assembly];
                }

                // reload heartbeat timer:
                Heartbeat[CHANNELS+Assembly] = fbSwitch[Assembly]::cpHeartbeatInterval;
            }
        } else {
            if(Buttons & 0x10u) {
                // SW5..SW8 activated:
                Duration = (unsigned) min(1u+Duration, BUTTON_LONG);
            } else {
                if((PreviousButtons & 0x10u) && Duration) {
                    // SW5..SW8 released:
                    if(Duration <= BUTTON_SHORT) {
                        // short button:
                        ProcessIsiButton(Assembly, TRUE, Buttons & 0x01u);
                    } else if(Duration >= BUTTON_LONG) {
                        // long button
                        ProcessIsiButton(Assembly, FALSE, Buttons & 0x01u);
                    }
                    Duration = 0;
                }
            }
        }
        ButtonDuration[Assembly] = Duration;
        Buttons = Next(Buttons);
        PreviousButtons = Next(PreviousButtons);
        Led <<= 1;
    }
}

//
//  At least one of the switch input feedback network variables has been updated. The switches use this
//  input to obtain the current state from the network, rather than making local assumptions. This allows
//  multiple pushbuttons to toggle multiple lights, or multiple dimmers to dim multiple lights, in a
//  meaningful and synchronized way.
//

when(nv_update_occurs(nviSwitchFb)) {
    unsigned Led;
    // when a NV update arrives, we ignore this unless it delivers new data. A closed loop
    // network variable connection scheme can cause network variable loops otherwise, where
    // updates caused by feedback network variables get repeatedly, possibly endlessly, 
    // propagated: 
    if(memcmp(&nviSwitchFb[nv_array_index], &nvoSwitch[nv_array_index], sizeof(SNVT_switch))) {
        if(IsEnabled(fbSwitch[nv_array_index]::global_index)) {

            // 1. follow-up with local light (if application-level coupling is needed):
            if(IsiIsRunning()) {
                if(!AlternativeMode[nv_array_index]) {
                    Led = (0x01u << nv_array_index);
                    SetLEDs(nviSwitchFb[nv_array_index].state ? Led : 0, Led);
                }
                nviLight[nv_array_index] = nviSwitchFb[nv_array_index];
                // update the light output network variable without propagating a network variable update.
                // see step #2, below, for a detailed discussion of the technique used here:

#ifdef _NEURONC
#   pragma disable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_off
#endif
#pragma relaxed_casting_on

                memcpy((void*)&nvoLightFb[nv_array_index], &nviSwitchFb[nv_array_index], sizeof(SNVT_switch));

#pragma relaxed_casting_off
#ifdef _NEURONC
#   pragma enable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_on
#endif

            }

            //  2. update the output NV, but do not propagate the new NV update. When a Neuron C application
            //	assigns a value to an output network variable, this variable gets automatically flagged for
            // 	propagation to the network at the end of the when-task (using the most recently assigned 
            //	value, unless the network variable is declared with 'sync' modifier). The network variable
            //	is not, however, automatically scheduled for propagation when the assignment is made through
            //	a pointer. We use this trick here to ensure the output network variable has the correct and
            //	current value without re-propagating the network variable  update. 
            // 	Note this implementation assumes a star-shaped feedback layout, as opposed to a daisy chain.

#ifdef _NEURONC
#   pragma disable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_off
#endif
#pragma relaxed_casting_on

            memcpy((void*)&nvoSwitch[nv_array_index], &nviSwitchFb[nv_array_index], sizeof(SNVT_switch));

#pragma relaxed_casting_off
#ifdef _NEURONC
#   pragma enable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_on
#endif

            // 3. reload heartbeat timer:
            Heartbeat[CHANNELS+nv_array_index] = fbSwitch[nv_array_index]::cpHeartbeatInterval;
        }
    }
}

//
//  At least one of the light input network variables has been updated. The light switches to the state
//  and level as advised with this network variable update, and repeats its current state and level on
//  the feedback output network variable, thereby allowing multiple pushbutton or dimmer switches to
//  synchronize with the light state.
//
when(nv_update_occurs(nviLight)) {
    unsigned Led;
    // when a NV update arrives, we ignore this unless it delivers new data. A closed loop
    // network variable connection scheme can cause network variable loops otherwise, where
    // updates caused by feedback network variables get repeatedly, possibly endlessly, 
    // propagated: 
    if(memcmp(&nviLight[nv_array_index], &nvoLightFb[nv_array_index], sizeof(SNVT_switch))) {
        if(IsEnabled(fbLight[nv_array_index]::global_index)) {

            // 1.   light is enabled and now gets a different state:
            if(!AlternativeMode[nv_array_index]) {
                Led = (0x01u << nv_array_index);
                SetLEDs(nviLight[nv_array_index].state ? Led : 0, Led);
            }

            // 2.   provide feedback
            nvoLightFb[nv_array_index] = nviLight[nv_array_index];

            // 3.	resynchronize local switch, if needed:
            if(IsiIsRunning()) {
                nviSwitchFb[nv_array_index] = nviLight[nv_array_index];

#ifdef _NEURONC
#   pragma disable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_off
#endif
#pragma relaxed_casting_on

                memcpy((void*)&nvoSwitch[nv_array_index], &nviLight[nv_array_index], sizeof(SNVT_switch));

#pragma relaxed_casting_off
#ifdef _NEURONC
#   pragma enable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_on
#endif

            }

            // 4.   reload heartbeat timer:
            Heartbeat[nv_array_index] = fbLight[nv_array_index]::cpHeartbeatInterval;
        }
    }
}

//
//  The thermometer is periodically read. The thermometer employs three configuration properties
//  that control network variable updates issued by this functional block: the heartbeat, the
//  throttle, and the minimum delta configuration properties.
//  The heartbeat interval determines a minimum update frequency: even if the reading has not
//  changed in a long time, the most recent value will be re-propagated at an interval determined
//  by the heartbeat timer.
//  The throttle period indicates a minimum time between two updates. In the event of rapidly changing
//  readings, this slows down network variable updates to a sustainable rate.
//  Finally, the minimum delta describes the minimum absolute difference between the most recent and
//  the current reading that is considered significant. New readings that vary less than the minimum
//  delta shall not be propagated on the network.
//
#ifdef INCLUDE_FLOAT
//  the flHundred constant is used with the float conversion, discussed below
const float_type flHundred = {0,0x42,1,0x48,0x0000}; /* 100.0 */
#endif  // INCLUDE_FLOAT

void ProcessThermometer(void) {
    SNVT_temp_p lTemperature;
#ifdef INCLUDE_FLOAT
    float_type flTemp;
#endif
    lTemperature = GetTemperature();

    if(IsEnabled(fbThermometer::global_index)
       && (abs(lTemperature - nvoTemperature) > nvoTemperature::cpDelta)) {
        // We have a new value, that varies enough from the most recent value to be
        // worth considering.
        // The thermometer implements both a heartbeat timer that ensures a minimum
        // rate of NV update propagation, and a throttle to ensure a maximum rate
        // of NV.
        // Here, we update the output network variables via pointers so that the new
        // values are stored, but the NVs won't get propagated automatically. Next,
        // we decide whether the updates can be propagated now, or schedule a future
        // propagation. Finally, the heartbeat timer is reset:

        // 1. update output NVs through pointers:

        //  note the following casts causes a, in this case, benign compiler warning
        // #463:  The 'const' attribute has been removed by cast operations
        // The second directive suppresses this warning

#ifdef _NEURONC
#   pragma disable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_off
#endif
#pragma relaxed_casting_on

        *((SNVT_temp_p*)&nvoTemperature) = lTemperature;

#ifdef INCLUDE_FLOAT
        // float conversion:
        // It is often both efficient and sufficient to execute the internal algorithms on
        // cardinal or fixed-point types such as SNVT_temp_p, and only to convert the inputs or
        // outputs to floating-point types if so needed. This is certainly the case here, so
        // all that is left to do is to convert the current temperature from the fixed-point
        // presentation (2 decimals) to a floating-point value. This requires a conversion of
        // the raw data to float, and a division by 100:
        fl_from_slong(lTemperature, &flTemp);
        fl_div(&flTemp, &flHundred, (float_type*)&nvoTemperatureF);
#endif

#pragma relaxed_casting_off
#ifdef _NEURONC
#   pragma enable_warning 463  // removal of 'const' attribute through cast
#else           
#   pragma warnings_on
#endif

        // 2. verify throttle. If we send now, reset the heartbeat. Let heartbeat
        // expire without resetting otherwise:
        if((fbNodeObject::nciNetConfig == CFG_LOCAL)
           || (fbThermometer::cpHeartbeatInterval-Heartbeat[2*CHANNELS] > fbThermometer::cpThrottle)) {
            // throttle has expired:
            propagate(nvoTemperature);
#ifdef INCLUDE_FLOAT
            propagate(nvoTemperatureF);
#endif
            Heartbeat[2*CHANNELS] = fbThermometer::cpHeartbeatInterval;
        }
    }
}

//
//  Network input for the buzzer. Notice how the same code gets used under two conditions here;
//  the two "when" clauses are logically OR'ed, and the related task body executes when either
//  condition is true.
when (nv_update_occurs(nviEnable))
when (nv_update_occurs(nviFrequency)) {
    unsigned long OutputValue;
    OutputValue = 0;

    if(IsEnabled(fbBuzzer::global_index)) {
        if(fbBuzzer::nviValue && fbBuzzer::nviEnable.state) {
            // compute the output value, assuming a clock frequency of 10MHz:
            // With a 10MHz crystal and a clock setting of 0 (zero) in the I/O
            // object declaration, the output value X is calculated as
            //
            //  X := 1 / (400ns * f)
            //
            //  With F = 10 * f (because UCPTfrequency implements a fixed-point type
            //  with one decimal), the output value becomes
            //
            //  X = 10^7 / (4 * F)
            //    = (10^3 * 10^4) / (4 * F)
            //    = (1000 * 2,500) / F
            //
            OutputValue = muldiv(1000ul, 2500ul, fbBuzzer::nviValue);
        }
        io_out(ioPiezzo, OutputValue);
    }
}

//
//  Processing of heartbeats and throttles
// 
//  Note this routine handles the application-generated heartbeats that are used when 
//  operating in a managed network. In a self-installed environment, where no network
//  integrator or network management tool is available to adjust the configuration 
//  properties that control the issue of network variable heartbeats, we use the heart-
//  beat service provided by the ISI engine. See IsiQueryHeartbeat() for more.
//
void ProcessHeartbeats() {
    unsigned Ch;
    // process switches and lamps first:
    for(Ch = 0; Ch < CHANNELS; ++Ch) {
        // lights:
        if(fbLight[Ch]::cpHeartbeatInterval && IsEnabled(fbLight[Ch]::global_index)) {
            // We are enabled and have a non-zero heartbeat interval, so let's execute
            // the heartbeat timer:
            if(--Heartbeat[Ch] == 0) {
                // timer expired. propagate NV and reload timer:
                propagate(fbLight[Ch]::nvoValueFb);
                Heartbeat[Ch] = fbLight[Ch]::cpHeartbeatInterval;
            }
        }
        // switches:
        if(fbSwitch[Ch]::cpHeartbeatInterval && IsEnabled(fbSwitch[Ch]::global_index)) {
            // We are enabled and have a non-zero heartbeat interval, so let's execute
            // the heartbeat timer:
            if(--Heartbeat[CHANNELS+Ch] == 0) {
                // timer expired. propagate NV and reload timer:
                propagate(fbSwitch[Ch]::nvoValue);
                Heartbeat[CHANNELS+Ch] = fbSwitch[Ch]::cpHeartbeatInterval;
            }
        }
    }
    // thermometer:
    if(fbThermometer::cpHeartbeatInterval && IsEnabled(fbThermometer::global_index)) {
        // HB ticking
        if(--Heartbeat[2u*CHANNELS] == 0) {
            // timer expired. propagate NV(s) and reload timer:
            propagate(fbThermometer::nvoHVACTemp);
#ifdef  INCLUDE_FLOAT
            propagate(fbThermometer::nvoFloatTemp);
#endif
            Heartbeat[2u*CHANNELS] = fbThermometer::cpHeartbeatInterval;
        }
    }
}

//  
// Heartbeats for self-installed operation. 
// 
// When the ISI engine is running and has been started with the isiFlagHeartbeat option, and the 
// application is linked with an ISI implementation that supports network variable heartbeats, ISI
// assists the application in determining the best possible time to issue a network variable heart-
// beat by inserting the heartbeat scheme into ISI's bandwidth controlled scheduler scheme. 
// Thus, the resulting heartbeat interval is not fixed, but a function of the network topology and
// size, as well as the complexity of the local application. 
// When the ISI engine decides a network variable heartbeat is due, it invokes the IsiQueryHeartbeat() 
// callback. By default, this routine always returns FALSE, indicating that no heartbeat was issued.
// The application may issue a single network variable heartbeat every time the IsiQueryHeartbeat() 
// callback override executes, for the single network variable indicated by global index with the 
// "nv" parameter. The application may use any application-specific means to do so, or may employ the 
// IsiIssueHeartbeat() utility function provided with the ISI library. If a heartbeat has been issued 
// no matter how, IsiQueryHeartbeat must return TRUE. 
// 
// The following example agrees to sending a heartbeat for every network variable suggested by the 
// ISI engine (bound output network variables only). More advanced logic may be implemented for 
// selective, application-specific, heartbeats.
// 
boolean IsiQueryHeartbeat(unsigned nv) {
    return IsiIssueHeartbeat(nv);
}

//
//  Periodic sampling of input
//
mtimer repeating tTick = 1000ul / TICKS_PER_SECOND;
far unsigned OneSec = TICKS_PER_SECOND;
far unsigned TenthSec = TICKS_PER_SECOND / 10u;

when(timer_expires(tTick)) {

    // Switches:
    ProcessButtons(GetButtons());

    // Thermometer:
    if(--OneSec == 0) {
        OneSec = TICKS_PER_SECOND;
        ProcessThermometer();
    }

    // 0.1 seconds. Process heartbeats
    if((fbNodeObject::nciNetConfig != CFG_LOCAL) && (--TenthSec == 0)) {
        TenthSec = TICKS_PER_SECOND / 10u;
        ProcessHeartbeats();
    }
}

//	wink event task. The WINK event is a standard network command, often send to
//	devices in order to verify the device's identity. Devices are not required to
//	honor this command, but it is highly recommended. When responding to the wink
//	command, devices should produce a visible or audible signal for a short while.
//	It is paramount that this signal must not be harmful.
when(wink) {
    unsigned Index, StoredPreviousLeds;
    StoredPreviousLeds = PreviousLEDs;

    for(Index = 0; Index != 255u; ++Index) {
        SetLEDs(Index, 0xFFu);
        delay(100);
    }
    SetLEDs(StoredPreviousLeds, 0xFF);
}

//  -----------------------------------------------------------------------
//  Node Object Implementation
//  -----------------------------------------------------------------------
when(nv_update_occurs(nviRequest)) {
    unsigned First, Last;
    SNVT_obj_status StatusOutput;
    boolean Copy;

    // The node object processes "requests" received via the nviRequest
    // variable. This variable describes the request in the object_request
    // field, and the selected object in the object_id field. The object_id
    // can be a specific object other than the node object, or 0 (zero) to
    // address all objects on the device with a single request.
    // This implementation only contains one object other than the node
    // object; all request apply to the fbSwitch object therefore.
    // See http://www.lonmark.org for interoperability guidelines.

    memset(&StatusOutput, 0, (unsigned)sizeof(StatusOutput));
    StatusOutput.object_id = nviRequest.object_id;

    if(nviRequest.object_id > fbBuzzer::global_index) {
        StatusOutput.invalid_id = 1;
    } else {
        if(nviRequest.object_id == 0) {
            First = 1;
            Last = fbBuzzer::global_index;
        } else {
            First = Last = (unsigned)nviRequest.object_id;
        }
        Copy = FALSE;

        if(nviRequest.object_request == RQ_NORMAL) {
            // Support for RQ_NORMAL is mandatory
            // Whichever state the object is in, return to normal operation:
            Disabled  = 0;
            StatusOutput.disabled = 0;
            Copy = TRUE;
        } else if(nviRequest.object_request == RQ_UPDATE_STATUS) {
            // Support for RQ_UPDATE_STATUS is mandatory
            // Report state for the selected object

            if(First == Last) {
                StatusOutput.disabled = !IsEnabled(First);
            } else {
                StatusOutput.disabled = 0;
                while(First <= Last) {
                    StatusOutput.disabled  |= !IsEnabled(First++);
                }
            }

        } else if(nviRequest.object_request == RQ_REPORT_MASK) {
            // Support for RQ_REPORT_MASK is mandatory
            // Report the object's capability:
            StatusOutput.disabled = StatusOutput.report_mask = 1;
        } else if(nviRequest.object_request == RQ_DISABLED) {
            // Supporting RQ_DISABLED is optional
            // Disable the selected object:

            Disabled |= FBLOCK_DISABLED;
            StatusOutput.disabled = 1;
            Copy = TRUE;
        } else if(nviRequest.object_request == RQ_ENABLE) {
            // Supporting RQ_ENABLE is optional
            // Enable the object:
            Disabled &= ~FBLOCK_DISABLED;
            StatusOutput.disabled = 0;
            Copy = TRUE;
        } else {
            // Command not supported. Indicate error:
            StatusOutput.invalid_request = 1;
        }

        if(Copy) while(First <= Last) {
                if(Disabled & FBLOCK_DISABLED) {
                    Disabled |= (FBLOCK_DISABLED << First);
                } else {
                    Disabled &= ~(FBLOCK_DISABLED << First);
                }
                ++First;
            }

        nvoStatus = StatusOutput;
    }
}

//
//  Linking with the ISI implementation (also notice when(reset))
//

//
//  A few definitions and variables:
//

// total number of assemblies supported by this application:
#define ASSEMBLY_FIRST_SWITCHLIGHTPAIR  0
#define ASSEMBLY_LAST_SWITCHLIGHTPAIR   (CHANNELS-1u)
#define ASSEMBLY_TEMPERATURE            (ASSEMBLY_LAST_SWITCHLIGHTPAIR+1u)
#define ASSEMBLY_BUZZER                 (ASSEMBLY_TEMPERATURE+1u)

// CHANNELS-pairs of Switch/Light combinations, plus one Thermometer and one Buzzer
#define ASSEMBLIES  (ASSEMBLY_BUZZER+1u)

// isiState tracks the state of the ISI engine for each assembly, thus allowing to decide
// which functions to invoke in any given context.
far IsiEvent isiState[ASSEMBLIES];

//  isiLed helps maintaining the state of the ISI-related LEDs LED5..LED8:
far unsigned isiLed[ASSEMBLIES];

//
//  ProcessIsiButton is called from the application (above) whenever an ISI connect button
//  has been activated for a short time (argument Constructive is TRUE), or for a long time (argument
//  Constructive is FALSE). Typically, short activation of related UI is a constructive operation:
//  create connections, accept enrollment, etc. Typically, long activation is a destructive operation:
//  cancel pending enrollment, delete existing connection, etc.
//  While the ProcessIsiButton does no more than calling straight-forward ISI interface functions,
//  the ProcessIsiButton function must decide which function to call based on the arguments passed
//  into the function as well as ISI state tracking information.
//
//  The Alternative flag is used to provide more user-interface options. On a device using a single
//  push-button per assembly, this flag should always be set to FALSE. Devices implementing a more
//  advanced user interface may use this flag to convey an alternative meaning. The implementation
//  provided here uses this flag to extend connections (Alternative = TRUE) rather than replace
//  connections (Alternative = FALSE), if so desired.
//
//  Note the Assembly number argument: Although applications are free to use any arbitrary (but
//  locally unique) assembly numbers, when using both application-specific overrides of ISI functions
//  as well as their default implementations, it is important to comply with the default assembly
//  number allocation scheme: by default, an assembly number equals the global index of the first
//  local network variable associated with the assembly.
//
void ProcessIsiButton(unsigned Assembly, boolean Constructive, boolean Alternative) {
    switch(isiState[Assembly]) {
        case isiPendingHost:
            // we are about to become a host for Assembly, but have not yet received any enrollment
            // acceptance messages (CSME):
            if(!Constructive) {
                IsiCancelEnrollment();
            }
            break;
        case isiPending:
            // we are engaged with an enrollment hosted elsewhere. The enrollment has been tentatively
            // accepted based on availability of device resources, but the application has not yet
            // accepted the enrollment. This happens here:
            if(Constructive) {
                if(Alternative) {
                    //Notice this optional feature requires network variable aliases; simple devices may
                    // choose never to extend, but always to replace, existing connections when accepting
                    // a new enrollment
                    IsiExtendEnrollment(Assembly);
                } else {
                    // accepting an enrollment in the standard way means to override and replace any
                    // previously existing connection information associated with this assembly. This is the
                    // typical behavior for most ISI-enabled devices.
                    IsiCreateEnrollment(Assembly);
                }
            }
            break;
        case isiApprovedHost:
            // an approved host is a connection host that has received at least one enrollment acceptance
            // (CSME) message.
            if(Constructive) {
                if(Alternative) {
                    //Notice this optional feature requires network variable aliases; simple devices may
                    // choose never to extend, but always to replace, existing connections when accepting
                    // a new enrollment
                    IsiExtendEnrollment(Assembly);
                } else {
                    // accepting an enrollment in the standard way means to override and replace any
                    // previously existing connection information associated with this assembly. This is the
                    // typical behavior for most ISI-enabled devices.
                    IsiCreateEnrollment(Assembly);
                }
            } else {
                //  Cancel the entire enrollment process:
                IsiCancelEnrollment();
            }
            break;
        case isiApproved:
            // an approved non-host is a device that accepted the open enrollment by making IsiExtendEnrollment of
            // IsiCreateEnrollment calls in the isiPending state. Such a device is currently waiting for a network
            // event (the arrival of a connection confirmation (CSMC) or cancellation (CSMX) message) to cancel or
            // complete the approved enrollment. The device may locally opt out of the previous acceptance:
            if(!Constructive) {
                IsiCancelEnrollment();
            }
            break;
        case isiNormal:
            //  in the normal state, the assembly is either unconnected or connected (call IsiIsConnected() to find
            //  out), but is not engaged in any pending enrollment. The ISI engine also is not engaged with any other
            //  state-changing operation, such as the domain ID acquisition process. At this state, it is safe to initiate
            //  new connections, or to delete existing ones:
            if(Constructive) {
                // Initiate a new connection enrollment (become the host for this):
                IsiOpenEnrollment(Assembly);
            } else {
                // Delete the connection
                if(Alternative) {
                    // The alternative behavior is to delete the entire connection, including any other devices also
                    // engaged with this connection:
                    IsiDeleteEnrollment(Assembly);
                } else {
                    // The standard destructive operation in this state is to opt out of all connections associated with
                    // the given assembly locally. If the device hosts the connection, this equals a IsiDeleteEnrollment()
                    // call. If the connection is hosted elsewhere, other participating devices will not be affected:
                    IsiLeaveEnrollment(Assembly);
                }
            }
            break;
        default:
            //  all other states: do nothing
            ;
    }   // end of switch(state)
}

//  uiTimer, the related timer task, and the ScheduleToClear functions are simple UI utilities: set a LED pattern, and call
//  ScheduleToClear() with the same pattern. This will (re-) trigger the UI timer. Upon expiry, this timer will clear the pattern.
//  (This is a simple emulation of a oneshot output model for the Mini Gizmo hardware.)
#define UI_TIMER    2;
stimer uiTimer;

far unsigned uiClearing;

void ScheduleToClear(unsigned Pattern) {
    uiClearing |= Pattern;
    uiTimer = UI_TIMER;
}

when(timer_expires(uiTimer)) {
    SetLEDs(0, uiClearing);
    uiClearing = 0;
}

//  altTimer provides the flashing of LED1..LED4 when in alternative mode
#define ALT_TIMER   250u
#define ALT_ON      0x02u   // set this bit in AlternativeMode to enable alternative mode
#define ALT_STATE   0x01u   // altTimer uses this to remember the current LED state

mtimer repeating altTimer = ALT_TIMER;

when(timer_expires(altTimer)) {
    unsigned Assembly, Led;
    Led = 0x01;
    for(Assembly = 0; Assembly < CHANNELS; ++Assembly) {
        if(AlternativeMode[Assembly] & ALT_ON) {
            SetLEDs(AlternativeMode[Assembly] & ALT_STATE ? 0 : Led, Led);
            AlternativeMode[Assembly] ^= ALT_STATE;
        }
        Led <<= 1u;
    }
}

//  The ConnectionAdverts table contains IsiCsmoData structures, one for each type of assembly hosted on this device which might become a connection host.
//  This data is used by the application-specific implementation of the IsiCreateCsmo and IsiGet[Next]Assembly functions, below.
#define CSMO_HOST_SWITCHLIGHT   0
#define CSMO_HOST_TEMPERATURE   1

static const IsiCsmoData Csmos[] = {
    // assemblies 0..3 (the switch/light pairs) offer this connection. 
    // 
    // Notice this is a manufacturer-specific enrollment (isiScopeManufacturer), specifying a manufacturer ID (9F.FF.FF) and a device class (05.01), but 
    // leaving the usage field blank (00) so as to support enrollment with devices with different usage, but otherwise identical configuration. 
    // 
    // group             direction           width profile   nvtype variant { Ack, Poll, Scope,                Application,                            Member }
    { ISI_DEFAULT_GROUP, isiDirectionVarious, 2,   0xFFFFul, 95u,   128u,   {  0,  0,    isiScopeManufacturer, { 0x9F, 0xFF, 0xFF, 0x05, 0x01, 0x00}, 1}},
    // 
    // assembly 4: the temperature sensor:
    //
    // This is an enrollment purely based on standard types; standard profile #9 (SFPThvacTempSensor). The specification of the Application fields (manufacturer ID,
    // device class and usage) is not required therefore, but this data must be provided nevertheless because we start the ISI engine with isiFlagExtended. We chose 
    // to blank the Application fields, however, since there is no need for this information with this enrollment, therefore supporting the widest possible range
    // of enrollment members.
    //
#ifdef INCLUDE_FLOAT
    // group             direction           width profile   nvtype variant { Ack, Poll, Scope,                Application,                            Member }
    { ISI_DEFAULT_GROUP, isiDirectionOutput, 2,    9,        0,     0,      {  0,  0,    isiScopeStandard,     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 1}}        
#else
    // group             direction           width profile   nvtype variant { Ack, Poll, Scope,                Application,                            Member }
    { ISI_DEFAULT_GROUP, isiDirectionOutput, 1,    9,        0,     0,      {  0,  0,    isiScopeStandard,     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 1}}        
#endif
    // assembly 5: the buzzer:
    // This table contains no record for this assembly, because we never initiate connections for this assembly. We might enroll to suitable connections
    // for this assembly, however. See IsiGet[Next]Assembly().
};

void IsiUpdateUserInterface(IsiEvent Event, unsigned Parameter) {
    if(Parameter == ISI_NO_ASSEMBLY && (Event == isiNormal || Event == isiCancelled)) {
        memset(isiState, 0, (unsigned)sizeof(isiState));
        memset(isiLed, 0, (unsigned)sizeof(isiLed));
        SetLEDs(0, 0xF0u);
    } else if(Event == isiRun) {
        SetLEDs(0x00u, 0xF0u);
    } else if(Event == isiWarm) {
        SetLEDs(0xF0u, 0xF0u);
        ScheduleToClear(0xF0u);
        IsiInitiateAutoEnrollment(&Csmos[CSMO_HOST_TEMPERATURE], ASSEMBLY_TEMPERATURE);
    } else if(Parameter < (unsigned)sizeof(isiState)) {
        isiState[Parameter] = Event;
    }
}

void IsiUpdateDiagnostics(IsiDiagnostic Event, unsigned Parameter) {
    if(Event == isiSubnetNodeDuplicate) {
        AlternativeMode[0] = ALT_ON;
    } else if(Event == isiSelectorDuplicate) {
        AlternativeMode[1] = ALT_ON;
    }
#pragma ignore_notused Parameter
}

void IsiCreateCsmo(unsigned Assembly, IsiCsmoData* pCsmoData) {
    if(Assembly <= ASSEMBLY_LAST_SWITCHLIGHTPAIR) {
        // this is one of the switch/light pairs:
        memcpy(pCsmoData, &Csmos[CSMO_HOST_SWITCHLIGHT], sizeof(IsiCsmoData));
    } else if(Assembly == ASSEMBLY_TEMPERATURE) {
        memcpy(pCsmoData, &Csmos[CSMO_HOST_TEMPERATURE], sizeof(IsiCsmoData));
    }
}

unsigned IsiGetAssembly(const IsiCsmoData* pIn, boolean Auto) {
    unsigned assembly;

    // this application does not accept connections requiring acknowledged service or polling:
    if(!pIn->Extended.Acknowledged && !pIn->Extended.Poll) {

        if(!Auto) {

            // Revision 2:
            // With ISI 3 introducing support for turnaround connections (provided the host is local), MgDemo must
            // suppress turnaround connections: MgDemo's switch/light pair assemblies implement four network 
            // variables each, enrolling with two selectors. Re-using selectors within each of these assemblies is 
            // required to support versatile N*MgSwitch + M*MgDemo + P*MgLight enrollment, but prevents the use of
            // turnaround enrollment and connections. To suppress turnaround enrollment, we detect whether one of
            // the switch/light pairs is currently becoming the host of a new connection. If this is the case, we
            // let this routine return ISI_NO_ASSEMBLY, indicating that no other applicable assembly exists.
            for(assembly=ASSEMBLY_FIRST_SWITCHLIGHTPAIR; assembly<=ASSEMBLY_LAST_SWITCHLIGHTPAIR; ++assembly) {
                if(isiState[assembly] == isiPendingHost || isiState[assembly] == isiApprovedHost) {
                    return ISI_NO_ASSEMBLY;
                }
            }

            // now test for the different acceptable connections:
            if(memcmp(pIn, &Csmos[CSMO_HOST_SWITCHLIGHT], sizeof(IsiCsmoData)) == 0) {
                // this matches the connection advertised for a switch/light pair:
                return ASSEMBLY_FIRST_SWITCHLIGHTPAIR;  // assembly# for first switch/light pair
            }

            if(pIn->Extended.Scope == isiScopeStandard && pIn->Extended.Member == 1 && pIn->Width == 2 && pIn->NvType == 95u) {
                if(pIn->Profile == 5  && pIn->Variant == 128u) {
                    // this is an offer made from MgLight:
                    return ASSEMBLY_FIRST_SWITCHLIGHTPAIR;
                }
                if(pIn->Profile == 3 && pIn->Variant == 0) {
                    // this is an offer made from MgSwitch, or any other standard SFPTclosedLoopSensor implementing SNVT_switch
                    return ASSEMBLY_FIRST_SWITCHLIGHTPAIR;
                }
            }
        } else {
            // we accept an automatic connection for the buzzer (MgKeyboard may connect to this):
            if(pIn->Extended.Scope == isiScopeStandard && pIn->Direction == isiDirectionOutput && pIn->Profile == 2 && pIn->NvType == 76u && pIn->Width == 1 && pIn->Variant == 0) {
                return ASSEMBLY_BUZZER;
            }
        }
    }
    return ISI_NO_ASSEMBLY;
}

unsigned IsiGetNextAssembly(const IsiCsmoData* pIn, boolean Auto, unsigned Assembly) {
    if(Assembly < ASSEMBLY_LAST_SWITCHLIGHTPAIR) {
        // we support a set of multiple similar assemblies ASSEMBLY_FIRST_SWITCHLIGHTPAIR .. ASSEMBLY_LAST_SWITCHLIGHTPAIR
        return Assembly + 1;
    }
    // all other assemblies are singletons:
    return ISI_NO_ASSEMBLY;

#pragma ignore_notused   pIn
#pragma ignore_notused   Auto
}

unsigned IsiGetNvIndex(unsigned Assembly, unsigned Offset) {
    if(Assembly <= ASSEMBLY_LAST_SWITCHLIGHTPAIR) {
        return Offset ? nvoSwitch[Assembly]::global_index : nvoLightFb[Assembly]::global_index;
    } else if(Assembly == ASSEMBLY_TEMPERATURE) {
        if(Offset == 0) {
            return nvoTemperature::global_index;
#ifdef INCLUDE_FLOAT
        } else if(Offset == 1) {
            return nvoTemperatureF::global_index;
#endif
        }
    } else if(Assembly == ASSEMBLY_BUZZER) {
        return nviFrequency::global_index;
    }
    return ISI_NO_INDEX;
}

unsigned IsiGetNextNvIndex(unsigned Assembly, unsigned Offset, unsigned PreviousIndex) {
    if(Assembly <= ASSEMBLY_LAST_SWITCHLIGHTPAIR) {
        if(PreviousIndex == nvoSwitch[Assembly]::global_index) {
            return nviLight[Assembly]::global_index;
        } else if(PreviousIndex == nvoLightFb[Assembly]::global_index) {
            return nviSwitchFb[Assembly]::global_index;
        }
    }
    return ISI_NO_INDEX;
#pragma ignore_notused  Offset
}

//
//  Keep the ISI engine running:
//
mtimer repeating IsiTicker = 1000ul / ISI_TICKS_PER_SECOND;

//  The device will return to factory defaults if the user activates the service pin continuously for 10 seconds:
#define SERVICE_PIN_ACTIVATION  (10u*ISI_TICKS_PER_SECOND)
unsigned    ServicePinActivation;

when(timer_expires(IsiTicker)) {
    unsigned Pattern, Assembly;

    // Call the ISI Tick function about four times per second:
    IsiTickS();

    // drive the ISI-related LEDs:
    Pattern = 0x10u;;
    for(Assembly = 0; Assembly < CHANNELS; ++Assembly) {
        switch(isiState[Assembly]) {
            case isiPending:
            case isiPendingHost:
                isiLed[Assembly] ^= Pattern;
                SetLEDs(isiLed[Assembly], Pattern);
                break;
            case isiApproved:
            case isiApprovedHost:
                SetLEDs(Pattern, Pattern);
                break;
            case isiImplemented:
            case isiCancelled:
            case isiDeleted:
                SetLEDs(Pattern, Pattern);
                ScheduleToClear(Pattern);
                isiState[Assembly] = isiLed[Assembly] = 0;
                break;
            case isiNormal:
                SetLEDs(0, Pattern);
                isiState[Assembly] = isiLed[Assembly] = 0;
                break;
        }
        Pattern <<= 1u;
    }

    // look after the service pin:
    if(service_pin_state()) {
        ++ServicePinActivation;
        if(ServicePinActivation > SERVICE_PIN_ACTIVATION) {
            OldNwrkCnfg = fbNodeObject::nciNetConfig = CFG_LOCAL;
#ifdef  SUPPORT_CCL
            // make sure the device also re-enters CENELEC configuration mode as part of
            // restoring factory defaults:
            firstStart = TRUE;
#endif  //  SUPPORT_CCL
            IsiReturnToFactoryDefaults();   // never returns!
        }
    } else {
        ServicePinActivation = 0;
    }
}



//
//  The ISI implementation library provides a default connection table, which allocates on-chip EEPROM
//  in the near segment. We override the default ISI connection table with our own version, assigning 
//  'far' memory to the connection table. Alternatively, rebuild this application to use a more recent
//  version of the system firmware, and to use the FT 3150 2K, PL 3150 2K or FT 5000 Smart Transceiver.
#define CONNECTION_TABLE_SIZE   24
eeprom far IsiConnection MyConnectionTable[CONNECTION_TABLE_SIZE];

unsigned IsiGetConnectionTableSize(void) {
    return CONNECTION_TABLE_SIZE;
}

const IsiConnection* IsiGetConnection(unsigned Index) {
    return &MyConnectionTable[Index];
}

void IsiSetConnection(IsiConnection* pConnection, unsigned Index) {
    MyConnectionTable[Index] = *pConnection;
}

//  The following override allows developing and debugging of this application in a managed NodeBuilder
//  development environment. See documentation for further, important, considerations related to debugging
//  ISI-enabled devices in a managed environment, and regarding the IsiSetDomain override in particular.
#ifdef _DEBUG
#ifndef _MINIKIT
void IsiSetDomain(domain_struct* pDomain, unsigned Index) {
    ;
#pragma  ignore_notused pDomain
#pragma  ignore_notused Index
}
#pragma ignore_notused IsiSetDomain
#endif
#endif

//simply blink the LEDS on the GIZMO board
/*
void blinkLEDS()
{
	unsigned Index, StoredPreviousLeds;
    StoredPreviousLeds = PreviousLEDs;

    for(Index = 0; Index != 255u; ++Index) {
        SetLEDs(Index, 0xFFu);
        delay(100);
    }
    SetLEDs(StoredPreviousLeds, 0xFF);
}
*/



//IO_3 output bit CTS;	//clear to send output
//IO_2 input bit RTS;		//request to send output
IO_8 input serial baud(4800) IOcharIn;
IO_10 output serial baud(4800) IOcharOut;
#define INPUT_BUFFER_SIZE 30
#define NUMBER_BUFFER_SIZE 11
#define STRING_LENGTH 20
struct {
	char inputBuffer[INPUT_BUFFER_SIZE];
}buffer;
char size[NUMBER_BUFFER_SIZE];
char character;
//char * pBuf;
char processBuffer=0,bufferCount=0,counter=0;
char msg_failed=0;
//read a line of 20 characters from the serial port

unsigned readLine(void)
{
	char * pointerToInputBuffer;//next char to store
	char * pointerToEndOfBuffer;//position of last byte
	char currentChar;//current char to process
	unsigned charCount;//# of chars
	unsigned counter; // get a full string
	pointerToInputBuffer = buffer.inputBuffer; //initialize buffer pointers
	pointerToEndOfBuffer = buffer.inputBuffer + sizeof(buffer.inputBuffer) - 1; // get the last byte position
	
	while(pointerToInputBuffer < pointerToEndOfBuffer)//prevents read past buffer
	{
		do
		{
			charCount = io_in(IOcharIn,pointerToInputBuffer,STRING_LENGTH);//read STRING_LENGTH chars
			watchdog_update();//avoid watch dog timeout
		}
		while(charCount==0); // if error or timeout try again
		
		
		for(counter=0;counter<STRING_LENGTH;counter++)
		{
			currentChar = *pointerToInputBuffer & 0x7F;	//discard any parity bit
			if(currentChar == '\r'){goto EXIT;} // exit if carriage return
			if( (currentChar == '\b') || (currentChar == 0x7F))
			{
				if(pointerToInputBuffer > buffer.inputBuffer)
				{
					pointerToInputBuffer--;
					io_out(IOcharOut,"\b \b",3);
				}
				continue;
			}
			
			if(currentChar < ' '){continue;}
			
			*pointerToInputBuffer = currentChar;
			
			io_out(IOcharOut,pointerToInputBuffer++,1); // echo this character
		}
	}
	
	EXIT:io_out(IOcharOut,"\r\n",2);
	*pointerToInputBuffer = '\0';
	return (unsigned)(pointerToInputBuffer - buffer.inputBuffer);//returns number of characters
}

/*
when( io_changes(RTS) to 1)
{
	pBuf = inputBuffer;
	io_out(CTS,1);
	do
	{
		(void)io_in(IOcharIn,pBuf,1);
		bufferCount++;
	}
	while(*pBuf++ != '\r' && bufferCount < INPUT_BUFFER_SIZE);//keep going while not CR
	processBuffer = 1;
}
*/
void clear_inputBuffer()
{//clear the input Buffer
	character = 0;
	while(character < INPUT_BUFFER_SIZE)
	{
		buffer.inputBuffer[character] = '\0';
		character++;
	}
}

#define INCOMING_BUFFER_SIZE 20
char incoming_buffer[INCOMING_BUFFER_SIZE];

void clear_incoming_buffer()
{
	char counter;
	for(counter=0;counter<INCOMING_BUFFER_SIZE;counter++)
	{
		incoming_buffer[counter] = '\0';
	}
}

when(msg_arrives) {
    if(IsiApproveMsg()) 
	{
        if(IsiProcessMsgS()) 
		{
            //  TODO: process unprocessed ISI messages here (if any)
            io_out(IOcharOut,"Receiving ISI:\r\n",15);
        }
    }
	else 
	{
		if(!msg_in.duplicate)
		{
			io_out(IOcharOut,"Receiving:",10);
			//character[0] = msg_in.data[0];
			//io_out(IOcharOut, character , 16);
			//io_out(IOcharOut, "\r\n" , 2);
			
			memcpy(incoming_buffer,msg_in.data,INCOMING_BUFFER_SIZE-1);//send a single number
			io_out(IOcharOut, incoming_buffer , msg_in.len);
			io_out(IOcharOut, "\r\n" , 2);
		}
    }
	clear_incoming_buffer();
}

#define CHECK_PERIOD 500
mtimer repeating readAndSend = CHECK_PERIOD;

when(timer_expires)
{
	bufferCount = readLine();
	if(bufferCount!=0)
	{
		processBuffer=1;
		io_out(IOcharOut,"Sending Message:",16);

		io_out(IOcharOut,buffer.inputBuffer,bufferCount);
		io_out(IOcharOut,"\r\n",2);
		io_out(IOcharOut,"Size:",5);
		
		size[0] = (bufferCount/100) + 48;
		size[1] = (bufferCount/10 % 10) +48;
		size[2] = (bufferCount % 10) + 48;
		size[3] = ' ';
		size[4] = 'B';
		size[5] = 'y';
		size[6] = 't';
		size[7] = 'e';
		size[8] = 's';
		size[9] = '\r';
		size[10] = '\n';
		io_out(IOcharOut,size,NUMBER_BUFFER_SIZE);
	}
	else{processBuffer = 0;}
}

#define MESSAGE_CODE 4
//create a message tag to use
msg_tag serialData;
when(processBuffer == 1)
{
	flush_wait(); // neccessary to do as we switch from explicit to implicit
	// control (i.e we use an if,while,...)
	//prepare the message
	msg_out.tag						 = serialData;
	msg_out.code					 = MESSAGE_CODE;
	character = buffer.inputBuffer[counter];
	//msg_out.data[0] = character;
	memcpy(msg_out.data,&buffer.inputBuffer,bufferCount);//send a string
	//memcpy(msg_out.data,&character,sizeof(character));//send a single number
	msg_send();
	while(!msg_succeeds(serialData))
	{
		watchdog_update();
		post_events();
		if(msg_fails(serialData))
		{
			msg_failed = 1;
			io_out(IOcharOut, "Message Failed" , 15);
			io_out(IOcharOut, "\r\n" , 2);
		}
	}
	if(msg_failed)
	{
		io_out(IOcharOut,"--Parts of Message failed to Send\r\n",35);
	}
	else
	{
		io_out(IOcharOut,"--Message Sent\r\n",16);
	}
	//clean up and reset data
	msg_failed = 0;
	bufferCount = 0;
	clear_inputBuffer();
	processBuffer = 0;
}

