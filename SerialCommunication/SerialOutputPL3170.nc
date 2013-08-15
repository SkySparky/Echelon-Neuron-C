//
//  Echelon Mini Kit, MGKeyboard Neuron C Example Application
//  Revision 3
//
//  This example demonstrates the implementation of an interoperable,
//  self-installed application within the limited space of a cost-
//  effective FT 3120 or PL 3120 Smart Transceiver single-chip solution.
//
//  IMPORTANT:
//  The MGKeyboard example uses a user-defined configuration property
//  type, UCPTfrequency. This data type is defined in user-defined
//  device resource files, which may be inspected and edited with the
//  NodeBuilder Resource Editor (also included with the Mini Evaluation
//  Kit). When compiling the MGKeyboard application, it is important
//  to set the standard program ID to a value that matches the user-
//  defined device resource files. Failure to do so will result in
//  compilation errors, as the compiler will be unable to recognize
//  the UCTPfrequency data type.
//
//  This application may be linked with any ISI library except the 
//  IsiCompactManual library. The smallest ISI library meeting the 
//  requirements of this application is IsiCompactAuto.lib.
//  When compiling for PL 3120 or FT 3120 evaluation boards, the 
//  IsiCompactAuto library is recommended.
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
//
// Revision 2:
// a)   Remove Application fields from IsiCsmoData - the enrollment is of 
//      standard scope, carrying Application data is optional but not necessary.
//		See MyCsmo variable for more.
// b)   Review of program IDs. Program IDs to use with this example application are
//      as follows:
//      PL 3150 Evaluation Board (CENELEC off): 9F.FF.FF.05.18.05.11.03
//      PL 3150 Evaluation Board (CENELEC on):  9F.FF.FF.05.18.05.10.03
//      FT 3150 Evaluation Board:               9F.FF.FF.05.18.04.04.03
//      PL 3120 Evaluation Board (CENELEC off): 9F.FF.FF.05.18.05.11.02
//      PL 3120 Evaluation Board (CENELEC on):  9F.FF.FF.05.18.05.10.02
//      FT 3120 Evaluation Board (801-0442-01): 9F.FF.FF.05.18.04.04.02
//      FT 3120 Evaluation Board              : 9F.FF.FF.05.18.04.04.04
// c)   Added support for the CENELEC Configuration Library (disabled by default)

// enable definition of the SUPPORT_CCL macro to enable support for the 
// CENELEC Configuration Library (CCL):
#define SUPPORT_CCL


#include <isi.h>
#include <mem.h>
#include <control.h>
#include <stdlib.h>
#include <snvt_cfg.h>
#include <string.h>


#ifdef  SUPPORT_CCL
#include <cenelec.h>
#endif  //  SUPPORT_CCL

// Define the number of alias table entries.
// For the most compact implementation, no alias table entries are required (set
// value to 0). To support versatile network variable connections when used in a
// managed network, specify a non-zero number of alias table entries.
// Because the device only hosts a single principal network variable, nvoFrequency,
// the number of alias table entries may be kept to a minimum (recommended for this
// example application: 3)
#pragma num_alias_table_entries  3

// Define the number of address table entries.
// One address table entry is sufficient for this application when used exclusively
// with the Mini Example suite, and in a self-installed environment. To support
// versatile connections in the general case, define a larger number of address table
// entries.
// Because the device only hosts a single principal network variable, nvoFrequency,
// the recommended size for the address table for this example application is one
// more than the number of alias table entries (1+3=4 for this example)
#pragma num_addr_table_entries  4

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
#endif  // _NEURONC

#pragma set_node_sd_string  "MGKeyboard"

// SCPTnwrkCnfg is a standard configuration property type, which is used to enable
// or disable self-installation algorithms such as the ISI implementation. Setting
// this property to CFG_EXTERNAL must disable all such code, allowing the device to
// be used in a managed network thus.
// Note that SCPTnwrkCnfg must be implemented as a configuration network variable 
// to ease transitions between self-installed and managed networks.
// Notice this property defaults to CFG_EXTERNAL. This is the default value 
// recommended by the LonMark Interoperability Guidelines, and is the best choice 
// when using the device in a managed network. See the when(reset) task, below, for
// more details about application start-up.
network input SCPTnwrkCnfg cp cp_info(reset_required) nciNetConfig = CFG_EXTERNAL;

// UCPTfrequency is a user-defined configuration property. The definition is contained
// in the MiniKit device resource files, and may be viewed and edited using the
// NodeBuilder Resource Editor tool.
// The property defines a frequency (in tenths of Hertz). The following cp_family
// declaration defines a configuration property array of eight elements, where each
// element defines the frequency corresponding to buttons SW1..SW8.
// In a self-installed environment, special configuration devices are required to make
// adjustments to configuration properties like the cpFrequency configuration property
// array. In a managed network, standard network tools can access and adjust these
// properties, allowing for easy configuration of the device. 
// The following cp_family declaration acts similar to a C-language typedef.
// Notice the cp_family declaration does not reserve memory and does not instantiate
// the property; see below for instantiation.
// See http://types.lonmark.org for more details of standard resource types.

#define OCTAVE  8
UCPTfrequency cp_family cpFrequency[OCTAVE] = {
    //  C(4)    D(4)    E(4)    F(4)    G(4)    A(4)    B(4)    C(5)
    2616ul, 2937ul, 3296ul, 3492ul, 3920ul, 4400ul, 4939ul, 5233ul
};

// Configuration properties may be implemented as configuration network variables,
// or in configuration files. The latter is often more efficient, and used in this
// example application. Three configuration files are pre-defined by LonMark International:
// The template file, the writable value file, and the read-only value file.
// The Neuron C Compiler automatically constructs these files, but the application must
// construct the directory of files (which may also list user-defined files).
// For more, see LonMark Application Layer Interoperability Guidelines
// (available at http://www.lonmark.org)
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
    0x20,  //  directory version
    2,     //  number of files: template file, and writable value file (no read-only file here)
    {
        { cp_template_file_len,         2 /* template file */,  cp_template_file},
        { cp_modifiable_value_file_len, 1 /* value file    */,  cp_modifiable_value_file},
    }
};

// nvoDirectory is implemented to publish the memory location of the file directory:
network output polled const SNVT_address nvoDirectory = (SNVT_address)&FileDirectory;
#pragma ignore_notused nvoDirectory

// nciNetConfig implements one member of the SCNTnwrkCnfg-based configuration property family
// declared earlier. This configuration property applies to the entire device (applies to the
// node object, if any):
device_properties {
    nciNetConfig
};


// nvoFrequency is the application's single principal network variable. The network
// variable provides the frequency corresponding to the currently activated button
// SW1..SW8, or 0 (zero) if no button is active.
// SNVT_freq_hz is a standard network variable type defined by LonMark International.
// See http://types.lonmark.org for more details of standard resource types.
network output bind_info(unackd_rpt) SNVT_freq_hz nvoFrequency;

// SFPTopenLoopSensor is a standard functional profile. The fbKeyboard functional
// block implements this profile, logically grouping the related network variable(s)
// and configuration properties.
// See http://types.lonmark.org for more details of standard resource types.
fblock SFPTopenLoopSensor {
    // associate network variables implemented in this device with
    // member network variables defined in the functional profile:
    nvoFrequency implements nvoValue;
} fbKeyboard external_name("Keyboard") fb_properties {
    // instantiate the UCPTfrequency-based configuration property array family declared
    // earlier.
    cpFrequency
};

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
    unsigned Buttons, Index;
    Buttons = 0xFFu;
    for(Index = 0; Index < MG_BUTTONS_DEBOUNCE; ++Index) {
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

unsigned PreviousLEDs = 0;  // used by SetLEDs, see there

void SetLEDs(unsigned LEDs, unsigned Mask) {
    // We may only want to set some LEDs, indicated by Mask. Bits outside the
    // mask are added from the previous pattern:
    LEDs |= PreviousLEDs & ~Mask;
    PreviousLEDs = LEDs;

    // LEDs are driven active low - the SetLEDs function handles the inversion
    // so that the application developer may think in positive logic:
    io_out(ioLEDs, ~LEDs);
    // stobe:
    io_out(ioLEDLd, 0);
    io_out(ioLEDLd, 1);
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
            // by chosing not to initialize these locals.
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
eeprom SCPTnwrkCnfg OldNwrkCnfg = CFG_NUL;

// when(reset) executes whenever the device resets and the application is allowed
// to execute. This is the case for configured devices, or for devices using
// the pragma run_unconfigured compiler directive. This directive is provided
// with the isi.h header file.
when(reset) {
    // we play a little with the LEDs and the buzzer. This is used to visually
    // detect a healthy node during manufacture and installation:
    SCPTnwrkCnfg cpNwrkConfig;

    // The following call to IsiPreStart() is only effective with target chips containing
    // the ISI core engine in ROM (such as the PL 3170 Smart Transceiver), and it does 
    // nothing otherwise. 
    IsiPreStart();

#ifdef  SUPPORT_CCL
    if(firstStart || service_pin_state()) {
        camOperation();
    }
#endif  //  SUPPORT_CCL

    SetLEDs(0xFF, 0xFFu);   // LEDs into defined state

    cpNwrkConfig = OldNwrkCnfg; 

    if(cpNwrkConfig == CFG_NUL) {
        // for the first application start, set nciNetConfig to CFG_LOCAL, thus allow the 
        // ISI engine to run by default: 
        ::nciNetConfig = CFG_LOCAL;
    }
    OldNwrkCnfg = ::nciNetConfig;

    if(::nciNetConfig == CFG_LOCAL) {
        if(cpNwrkConfig == CFG_EXTERNAL) {
            //	The application has just returned into the self-installed environment. 
            //  Make sure to re-initialize the entire ISI engine:
            IsiReturnToFactoryDefaults();   // Call NEVER returns! (resets the device)
        }
        //  We are in a self-installed network:
        //  Start the ISI engine using the basic messaging set; see isiFlagExtended in the IsiFlags enumeration,
        //	detailed in the ISI Programmer's Guide, for more details.
        IsiStartS(isiFlagNone);
    }

    SetLEDs(0, 0xFFu);
}

mtimer repeating kbTick = 50u;  // timer to sample the buttons
unsigned PreviousButtons = 0;   // used by when(timer_expires(kbTick))

//  The device will return to factory defaults if the user activates the service pin continuously for 10 seconds:
#define SERVICE_PIN_ACTIVATION  (200u)
unsigned ServicePinActivation = 0;

when(timer_expires(kbTick)) {
    unsigned Buttons, Index, Pattern;

    Buttons = GetButtons();

    // detect and process changes:
    if(PreviousButtons != Buttons) {
        PreviousButtons = Buttons;
        Pattern = 0x01u;
        for(Index = 0; Index < 8u; ++Index) {
            // when iterating over each button SW1..SW8, we take advantage of
            // the fact that each when-task is a critical section: multiple
            // assignments to a network variable within a single when-task
            // will just overwrite the previous assignment; the last value
            // assigned will be propagated when the when-task completes (if the
            // network variable is bound).
            // The sync modifier may be used with the network variable declaration,
            // or the propagate() standard library function may be called, to
            // alter this behavior.
            if(Buttons & Pattern) {
                SetLEDs(Pattern, Pattern);
                nvoFrequency = fbKeyboard::cpFrequency[Index];
                // if multiple buttons are pressed, we stop with the first one
                // detected:
                break;
            } else {
                SetLEDs(0, Pattern);
                nvoFrequency = 0;
            }
            Pattern <<= 1u;
        }
    }

    // look after the service pin:
    if(service_pin_state()) {
        ++ServicePinActivation;
        if(ServicePinActivation > SERVICE_PIN_ACTIVATION) {
            // when the service pin has been pressed and held for a long time
            // (recommended: 10 seconds), the device resets and returns to
            // factory defaults.
            // The application is responsible for resetting configuration property
            // values and application variables as needed:
            OldNwrkCnfg = ::nciNetConfig = CFG_LOCAL;

#ifdef  SUPPORT_CCL
            // make sure the device also re-enters CENELEC configuration mode as part of
            // restoring factory defaults:
            firstStart = TRUE;
#endif  //  SUPPORT_CCL
            // Re-initialize the ISI engine. Notice the function completes with
            // resetting the device; the function never returns therefore.
            IsiReturnToFactoryDefaults();   // never returns!
        }
    } else {
        ServicePinActivation = 0;
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


//
//  ISI
//

mtimer repeating isiTick = 1000ul / ISI_TICKS_PER_SECOND;   // ISI engine timer

when(timer_expires(isiTick)) {
    // call into the ISI engine 4 times per second:
    IsiTickS();
}



//  MyCsmo defines the enrollment details for the automatic ISI network variable connection
//  advertised by this device.
// 
//  Revision 2:
//  Removed explicit initializers for application fields. As per the rules of the C language, these
//  will default to zero and we must provide storage, but since this application doesn't start the 
//  ISI engine with the isiFlagExtended flag, they won't be transmitted. The following example shows
//	the minimum IsiCsmoData intialization required:
static const IsiCsmoData MyCsmo = {
    // group             direction           width profile   nvtype variant 
    ISI_DEFAULT_GROUP, isiDirectionOutput, 1,    2,        76u,   0
};  

// The IsiUpdateUserInterface callback gets invoked any time a state change occurs that
// needs reflecting on the user interface. This implementation ignores most of these
// calls, but evaluates the isiWarm event. This event is fired when the required minimum
// time has passed following a reset to allow for opening of automatic enrollment.
void IsiUpdateUserInterface(IsiEvent Event, unsigned Parameter) {
    if(Event == isiWarm && !IsiIsConnected(0)) {
        // we waited long enough and we are not connected already, so let's open an
        // automatic connection:
        IsiInitiateAutoEnrollment(&MyCsmo, 0);
    }
#pragma ignore_notused Parameter
}

// The IsiCreateCsmo callback gets called whenever the ISI engine needs to handle a
// message containing IsiCsmoData.
// Because this device only implements a single assembly (with number 0), we can ignore
// the Assembly parameter in this case
void IsiCreateCsmo(unsigned Assembly, IsiCsmoData* pCsmoData) {
    memcpy(pCsmoData, &MyCsmo, sizeof(IsiCsmoData));
#pragma ignore_notused  Assembly
}

// Override IsiGetNvIndex with application-specific implementation. This often results in
// smaller memory requirements compared to using the generic default routines provided with
// the ISI implementation.
// Because this example application refuses all incoming open enrollment messages and only
// initiates an enrollment for a single assembly, we can ignore both parameters here and
// provide the index of the only network variable that might be connected with ISI:
unsigned IsiGetNvIndex(unsigned Assembly, unsigned Offset) {
    return nvoFrequency::global_index;
#pragma ignore_notused  Assembly
#pragma ignore_notused  Offset
}

// Override IsiGetAssembly / IsiGetNextAssembly with application-specific implementations.
// This often results in smaller memory requirements compared to using the generic default
// routines provided with the ISI implementation.
// This example application does not accept any enrollment messages. This is implemented
// with the following two overrides: whatever the incoming IsiCsmoData indicates, the
// application always signals "no applicable assembly":
unsigned IsiGetAssembly(const IsiCsmoData* pCsmoData, boolean Auto) {
    // we never accept open enrollment messages
    return ISI_NO_ASSEMBLY;
#pragma ignore_notused pCsmoData
#pragma ignore_notused Auto
}

unsigned IsiGetNextAssembly(const IsiCsmoData* pCsmoData, boolean Auto, unsigned Assembly) {
    return ISI_NO_ASSEMBLY;
#pragma ignore_notused pCsmoData
#pragma ignore_notused Auto
#pragma ignore_notused  Assembly
}

//
// The ISI implementation library provides a default connection table. Since we only have
// one network variable, we may never join more than one connection. A single connection
// table entry is sufficient therefore:
eeprom IsiConnection MyConnectionTable;

unsigned IsiGetConnectionTableSize(void) {
    return 1u;
}

const IsiConnection* IsiGetConnection(unsigned Index) {
    return &MyConnectionTable;
#pragma ignore_notused Index
}

void IsiSetConnection(IsiConnection* pConnection, unsigned Index) {
    MyConnectionTable = *pConnection;
#pragma ignore_notused Index
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

#define INCOMING_BUFFER_SIZE 40
IO_10 output serial baud(4800) IOcharOut;
char incoming_buffer[INCOMING_BUFFER_SIZE];

void clear_incoming_buffer()
{
	char character;
	character = 0;
	while(character < INCOMING_BUFFER_SIZE)
	{
		incoming_buffer[character] = '\0';
		character++;
	}
}
char length;
when(msg_arrives) 
{
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
		if(!msg_in.duplicate)//only print the message once
		{
			io_out(IOcharOut,"Receiving:",10);
			//character[0] = msg_in.data[0];
			//io_out(IOcharOut, character , 16);
			//io_out(IOcharOut, "\r\n" , 2);
			length = msg_in.len;
			memcpy(incoming_buffer,msg_in.data,INCOMING_BUFFER_SIZE-1);//send a single number
			io_out(IOcharOut, incoming_buffer , length);
			io_out(IOcharOut, "\r\n" , 2);
		}
		msg_free();
    }
}

mtimer repeating clean_buffer = 100;
when(timer_expires(clean_buffer))
{
	clear_incoming_buffer();
}