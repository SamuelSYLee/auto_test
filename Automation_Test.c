#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <phidget22.h>
#include "PhidgetHelperFunctions.h"
#include <time.h>
#include <stdbool.h>
#pragma warning(disable: 4996)

#define _TEST_MODE_ENABLE		0	//Test mode or not
#define _TIME_BASE_ROTATION		5	//Base time for rotation servo, in second
#define _TIME_BASE_ACTUATOR		3	//Base time for linear actuator, in second
#define _TIME_BASE_RELAY		2	//Base time for relay board, in second

FILE * fp;
bool flag_wait_display;
bool flag_wait_change_Relay;
bool flag_wait_change_Actuator;
bool flag_wait_change_Rotate;
bool flag_attach_hum;
bool flag_attach_psr;
bool flag_attach_CT;
bool flag_relay_off;
bool flag_clockwise;
bool flag_act_push;

time_t nowtime;
time_t exetime;
time_t starttime;
time_t endtime;
struct tm * timeinfo;
struct tm * tarTime;

clock_t dataclock;
clock_t startclock;
clock_t endclock;
clock_t nowclock;

char status_servo[20];
char status_act[10];
char status_relay[10];
char CW[20] = "Clockwise";
char CCW[20] = "Counterclockwise";
char PUSH[10] = "ActPush";
char PULL[10] = "ActPull";
char RON[10] = "RealyOn";
char ROFF[10] = "RealyOff";

int32_t cnt_data;

PhidgetVoltageInputHandle ch_V = NULL;			//Voltage Sensor
PhidgetVoltageInputHandle ch_CT = NULL;			//DC Current Transducer
PhidgetVoltageRatioInputHandle ch_I = NULL;		//Current Sensor
PhidgetVoltageRatioInputHandle ch_H = NULL;		//Humidity Sensor
PhidgetVoltageRatioInputHandle ch_P = NULL;		//Air Pressure Sensor
PhidgetTemperatureSensorHandle ch_T = NULL;		//Temperature Sensor
PhidgetRCServoHandle ch_R = NULL;				//Rotation Servo
PhidgetDCMotorHandle ch_L = NULL;				//Linear Actuators
PhidgetDigitalOutputHandle ch_B = NULL;			//Relay Board

double valueVolt;
double valueCT;
double valueCurr;
double valueHum;
double valueAP;
double valueTemp;
Phidget_UnitInfo sensorUnit_Volt;
Phidget_UnitInfo sensorUnit_CT;
Phidget_UnitInfo sensorUnit_Curr;
Phidget_UnitInfo sensorUnit_Hum;
Phidget_UnitInfo sensorUnit_AP;
PhidgetReturnCode prc_Volt;
PhidgetReturnCode prc_Unit_V;
PhidgetReturnCode prc_CT;
PhidgetReturnCode prc_Unit_CT;
PhidgetReturnCode prc_Curr;
PhidgetReturnCode prc_Unit_I;
PhidgetReturnCode prc_Hum;
PhidgetReturnCode prc_Unit_H;
PhidgetReturnCode prc_AP;
PhidgetReturnCode prc_Unit_P;
PhidgetReturnCode prc_Temp;

/**
* Configures the device's DataInterval and ChangeTrigger.
* Displays info about the attached Phidget channel.
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param ph The Phidget channel that fired the attach event
* @param *ctx Context pointer. Used to pass information to the event handler.
*/
static void CCONV onAttachHandler(PhidgetHandle ph, void *ctx) {
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
	Phidget_ChannelClass channelClass;
	Phidget_ChannelSubclass channelSubclass;
	Phidget_DeviceClass deviceClass;
	char* channelClassName;
	int32_t serialNumber;
	int32_t hubPort;
	int32_t channel;

	//If you are unsure how to use more than one Phidget channel with this event, we recommend going to
	//www.phidgets.com/docs/Using_Multiple_Phidgets for information

	printf("\nAttach Event: ");
	/*
	* Get device information and display it.
	*/
	prc = Phidget_getDeviceSerialNumber(ph, &serialNumber);
	CheckError(prc, "Getting DeviceSerialNumber", (PhidgetHandle *)&ph);

	prc = Phidget_getChannel(ph, &channel);
	CheckError(prc, "Getting Channel", (PhidgetHandle *)&ph);

	prc = Phidget_getChannelClassName(ph, &channelClassName);
	CheckError(prc, "Getting Channel Class Name", (PhidgetHandle *)&ph);

	prc = Phidget_getDeviceClass(ph, &deviceClass);
	CheckError(prc, "Getting Device Class", (PhidgetHandle *)&ph);

	if (deviceClass == PHIDCLASS_VINT) {
		prc = Phidget_getHubPort(ph, &hubPort);
		CheckError(prc, "Getting HubPort", (PhidgetHandle *)&ph);

		printf("\n\t-> Channel Class: %s\n\t-> Serial Number: %d\n\t-> Hub Port: %d\n\t-> Channel %d\n\n", channelClassName, serialNumber, hubPort, channel);
	}
	else { //Not VINT
		printf("\n\t-> Channel Class: %s\n\t-> Serial Number: %d\n\t-> Channel %d\n\n", channelClassName, serialNumber, channel);
	}

	/*
	*	Set the DataInterval inside of the attach handler to initialize the device with this value.
	*	DataInterval defines the minimum time between VoltageRatioChange events.
	*	DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
	*/
	//printf("\tSetting DataInterval to 1000ms\n");
	//prc = PhidgetTemperatureSensor_setDataInterval((PhidgetTemperatureSensorHandle)ph, 1000);
	//CheckError(prc, "Setting DataInterval", (PhidgetHandle *)&ph);

	/*
	*	Set the VoltageRatioChangeTrigger inside of the attach handler to initialize the device with this value.
	*	VoltageRatioChangeTrigger will affect the frequency of VoltageRatioChange events, by limiting them to only occur when
	*	the voltage ratio changes by at least the value set.
	*/
	printf("\tSetting ChangeTrigger to 0.0\n");
	prc = PhidgetVoltageRatioInput_setVoltageRatioChangeTrigger((PhidgetVoltageRatioInputHandle)ph, 0.0);
	//CheckError(prc, "Setting ChangeTrigger", (PhidgetHandle *)&ph);
	prc = PhidgetVoltageInput_setVoltageChangeTrigger((PhidgetVoltageInputHandle)ph, 0.0);
	//CheckError(prc, "Setting ChangeTrigger", (PhidgetHandle *)&ph);
	prc = PhidgetTemperatureSensor_setTemperatureChangeTrigger((PhidgetTemperatureSensorHandle)ph, 0.0);
	//CheckError(prc, "Setting ChangeTrigger", (PhidgetHandle *)&ph);

	/**
	* Set the SensorType inside of the attach handler to initialize the device with this value.
	* You can find the appropriate SensorType for your sensor in its User Guide and the VoltageRatioInput API
	* SensorType will apply the appropriate calculations to the voltage ratio reported by the device
	* to convert it to the sensor's units.
	* SensorType can only be set for Sensor Port voltageRatio inputs(VINT Ports and Analog Input Ports)
	*/
	prc = Phidget_getChannelClass(ph, &channelClass);
	CheckError(prc, "Getting ChannelClass", (PhidgetHandle *)&ph);

	prc = Phidget_getChannelSubclass(ph, &channelSubclass);
	CheckError(prc, "Getting ChannelSubClass", (PhidgetHandle *)&ph);

	if (channelSubclass == PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT) {
		printf("\tSetting VoltageRatio SensorType\n");

		if (!flag_attach_hum && !flag_attach_psr)
		{
#if _TEST_MODE_ENABLE > 0
			prc = PhidgetVoltageRatioInput_setSensorType((PhidgetVoltageRatioInputHandle)ph, 0x0);	//Edited by Samuel 2018.10.17, SENSOR_TYPE_1122_DC
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#else
			prc = PhidgetVoltageRatioInput_setSensorType((PhidgetVoltageRatioInputHandle)ph, SENSOR_TYPE_1122_DC);	//Edited by Samuel 2018.10.17, SENSOR_TYPE_1122_DC
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#endif
			flag_attach_hum = true;
		}
		else if (flag_attach_hum && !flag_attach_psr)
		{
			prc = PhidgetVoltageRatioInput_setSensorType((PhidgetVoltageRatioInputHandle)ph, SENSOR_TYPE_3130);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);

			flag_attach_psr = true;
		}
		else
		{
			prc = PhidgetVoltageRatioInput_setSensorType((PhidgetVoltageRatioInputHandle)ph, SENSOR_TYPE_1139);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
		}
	}

	if (channelSubclass == PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT) {
		printf("\tSetting Voltage SensorType\n");

		if (!flag_attach_CT)
		{
#if _TEST_MODE_ENABLE > 0
			prc = PhidgetVoltageInput_setSensorType((PhidgetVoltageInputHandle)ph, 0x0);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#else
			prc = PhidgetVoltageInput_setSensorType((PhidgetVoltageInputHandle)ph, SENSOR_TYPE_1135);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#endif
			flag_attach_CT = true;
		}
		else
		{
#if _TEST_MODE_ENABLE > 0
			prc = PhidgetVoltageInput_setSensorType((PhidgetVoltageInputHandle)ph, 0x0);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#else
			prc = PhidgetVoltageInput_setSensorType((PhidgetVoltageInputHandle)ph, SENSOR_TYPE_3587);
			CheckError(prc, "Setting SensorType", (PhidgetHandle *)&ph);
#endif
		}

	}

	if (channelClass == PHIDCHCLASS_RCSERVO)
	{
		printf("\tSetting RC Servo\n");

		/*
		* Set a TargetPosition inside of the attach handler to initialize the servo's starting position.
		* TargetPosition defines position the RC Servo will move to.
		* TargetPosition can be set to any value from MinPosition to MaxPosition.
		*/
		prc = PhidgetRCServo_setTargetPosition((PhidgetRCServoHandle)ph, 80);
		//CheckError(prc, "Setting Target Position", (PhidgetHandle *)&ph);

		/*
		* Engage the RCServo inside of the attach handler to allow the servo to move to its target position
		* The servo will only track a target position if it is engaged.
		* Engaged can be set to TRUE to enable the servo, or FALSE to disable it.
		*/
		prc = PhidgetRCServo_setEngaged((PhidgetRCServoHandle)ph, 1);
		//CheckError(prc, "Setting Engaged", (PhidgetHandle *)&ph);
	}
}

/**
* Displays info about the detached Phidget channel.
* Fired when a Phidget channel with onDetachHandler registered detaches
*
* @param ph The Phidget channel that fired the detach event
* @param *ctx Context pointer. Used to pass information to the event handler.
*/
static void CCONV onDetachHandler(PhidgetHandle ph, void *ctx) {
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
	Phidget_DeviceClass deviceClass;
	char* channelClassName;
	int32_t serialNumber;
	int32_t hubPort;
	int32_t channel;

	//If you are unsure how to use more than one Phidget channel with this event, we recommend going to
	//www.phidgets.com/docs/Using_Multiple_Phidgets for information

	printf("\nDetach Event: ");
	/*
	* Get device information and display it.
	*/
	prc = Phidget_getDeviceSerialNumber(ph, &serialNumber);
	CheckError(prc, "Getting DeviceSerialNumber", (PhidgetHandle *)&ph);

	prc = Phidget_getChannel(ph, &channel);
	CheckError(prc, "Getting Channel", (PhidgetHandle *)&ph);

	prc = Phidget_getChannelClassName(ph, &channelClassName);
	CheckError(prc, "Getting Channel Class Name", (PhidgetHandle *)&ph);

	prc = Phidget_getDeviceClass(ph, &deviceClass);
	CheckError(prc, "Getting Device Class", (PhidgetHandle *)&ph);

	if (deviceClass == PHIDCLASS_VINT) {
		prc = Phidget_getHubPort(ph, &hubPort);
		CheckError(prc, "Getting HubPort", (PhidgetHandle *)&ph);

		printf("\n\t-> Channel Class: %s\n\t-> Serial Number: %d\n\t-> Hub Port: %d\n\t-> Channel %d\n\n", channelClassName, serialNumber, hubPort, channel);
	}
	else { //Not VINT
		printf("\n\t-> Channel Class: %s\n\t-> Serial Number: %d\n\t-> Channel %d\n\n", channelClassName, serialNumber, channel);
	}
}

/**
* Writes Phidget error info to stderr.
* Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
*
* @param ph The Phidget channel that fired the error event
* @param *ctx Context pointer. Used to pass information to the event handler.
* @param errorCode the code associated with the error of enum type Phidget_ErrorEventCode
* @param *errorString string containing the description of the error fired
*/
static void CCONV onErrorHandler(PhidgetHandle ph, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString) {

	fprintf(stderr, "[Phidget Error Event] -> %s (%d)\n", errorString, errorCode);
}

/**
* Prints descriptions of the available events for this class
*/
void PrintEventDescriptions() {

	printf("\n--------------------\n"
		"\n  | VoltageRatio change events will call their associated function every time new voltage ratio data is received from the device.\n"
		"  | The rate of these events can be set by adjusting the DataInterval for the channel.\n");

	printf(
		"\n  | Sensor change events contain the most recent sensor value received from the device.\n"
		"  | Sensor change events will occur instead of VoltageRatio change events if the SensorType is changed from the default.\n"
		"  | Press ENTER once you have read this message.");
	getchar();

	printf("\n--------------------\n");
}

/**
* Initialize variables
*/
void Initialization()
{
	flag_wait_display = true;
	flag_wait_change_Relay = true;
	flag_wait_change_Rotate = true;
	flag_wait_change_Actuator = true;

	flag_attach_hum = false;
	flag_attach_psr = false;
	flag_attach_CT = false;

	cnt_data = 1;
	flag_clockwise = false;
	flag_act_push = true;
	flag_relay_off = false;

	nowtime = 0;
	exetime = 0;
	starttime = 0;
	endtime = 0;

	dataclock = 0;
	startclock = 0;
	endclock = 0;
	nowclock = 0;

	valueVolt = 0;
	valueCurr = 0;
	valueHum = 0;
	valueAP = 0;
	valueTemp = 0;
}

/**
* Ask for the total executing time
*/
void AskExecuteTime()
{
	printf("\n--------------------------------------\n");

	while (exetime <= 0)
	{
		printf("\nPlease type in the total executing time in second, must larger than 0s = ");
		scanf("%lld", &exetime);
	}

	while (dataclock < 100)
	{
		printf("\nPlease type in the data interval time in millisecond, no less than 100ms = ");
		scanf("%ld", &dataclock);
	}

	printf("\n--------------------------------------\n");

	starttime = time(NULL);
	endtime = exetime + starttime;
	startclock = clock();
	endclock = (clock_t)exetime * CLOCKS_PER_SEC + startclock;
}


/**
* Creates, configures, and opens a VoltageRatioInput channel.
* Displays voltage ratio change events for 10 seconds
* Closes out VoltageRatioInput channel
*
* @return 0 if the program exits successfully, 1 if it exits with errors.
*/
int main() {
	ChannelInfo channelInfo_V; //Information from AskForDeviceParameters(). May be removed when hard-coding parameters.
	ChannelInfo channelInfo_CT;
	ChannelInfo channelInfo_I;
	ChannelInfo channelInfo_H;
	ChannelInfo channelInfo_P;
	ChannelInfo channelInfo_T;
	ChannelInfo channelInfo_R;
	ChannelInfo channelInfo_L;
	ChannelInfo channelInfo_B;
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call

	/*
	* Allocate a new Phidget Channel object
	*/

	prc = PhidgetVoltageInput_create(&ch_V);
	prc = PhidgetVoltageInput_create(&ch_CT);
	prc = PhidgetVoltageRatioInput_create(&ch_I);
	prc = PhidgetVoltageRatioInput_create(&ch_H);
	prc = PhidgetVoltageRatioInput_create(&ch_P);
	prc = PhidgetDCMotor_create(&ch_L);
	prc = PhidgetTemperatureSensor_create(&ch_T);
	prc = PhidgetRCServo_create(&ch_R);
	prc = PhidgetDigitalOutput_create(&ch_B);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_V);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_CT);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_I);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_H);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_P);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_T);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_R);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_L);
	CheckError(prc, "Creating Channel", (PhidgetHandle *)&ch_B);

	/*
	* Set addressing parameters to specify which channel to open
	*/

	//You can safely remove this line and hard-code the parameters to make the program can start without user input
	AskForDeviceParameters(&channelInfo_V, (PhidgetHandle *)&ch_V);
	AskForDeviceParameters(&channelInfo_CT, (PhidgetHandle *)&ch_CT);
	AskForDeviceParameters(&channelInfo_I, (PhidgetHandle *)&ch_I);
	AskForDeviceParameters(&channelInfo_H, (PhidgetHandle *)&ch_H);
	AskForDeviceParameters(&channelInfo_P, (PhidgetHandle *)&ch_P);
	AskForDeviceParameters(&channelInfo_T, (PhidgetHandle *)&ch_T);
	AskForDeviceParameters(&channelInfo_R, (PhidgetHandle *)&ch_R);
	AskForDeviceParameters(&channelInfo_L, (PhidgetHandle *)&ch_L);
	AskForDeviceParameters(&channelInfo_B, (PhidgetHandle *)&ch_B);


	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_V, channelInfo_V.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_V);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_CT, channelInfo_CT.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_I, channelInfo_I.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_I);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_H, channelInfo_H.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_H);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_P, channelInfo_P.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_P);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_T, channelInfo_T.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_T);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_R, channelInfo_R.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_R);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_L, channelInfo_L.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_L);
	prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch_B, channelInfo_B.deviceSerialNumber);
	CheckError(prc, "Setting DeviceSerialNumber", (PhidgetHandle *)&ch_B);

	prc = Phidget_setHubPort((PhidgetHandle)ch_V, channelInfo_V.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_V);
	prc = Phidget_setHubPort((PhidgetHandle)ch_CT, channelInfo_CT.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setHubPort((PhidgetHandle)ch_I, channelInfo_I.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_I);
	prc = Phidget_setHubPort((PhidgetHandle)ch_H, channelInfo_H.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_H);
	prc = Phidget_setHubPort((PhidgetHandle)ch_P, channelInfo_P.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_P);
	prc = Phidget_setHubPort((PhidgetHandle)ch_T, channelInfo_T.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_T);
	prc = Phidget_setHubPort((PhidgetHandle)ch_R, channelInfo_R.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_R);
	prc = Phidget_setHubPort((PhidgetHandle)ch_L, channelInfo_L.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_L);
	prc = Phidget_setHubPort((PhidgetHandle)ch_B, channelInfo_B.hubPort);
	CheckError(prc, "Setting HubPort", (PhidgetHandle *)&ch_B);

	prc = Phidget_setIsHubPortDevice((PhidgetHandle)ch_V, channelInfo_V.isHubPortDevice);
	CheckError(prc, "Setting IsHubPortDevice", (PhidgetHandle *)&ch_V);
	prc = Phidget_setIsHubPortDevice((PhidgetHandle)ch_CT, channelInfo_CT.isHubPortDevice);
	CheckError(prc, "Setting IsHubPortDevice", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setIsHubPortDevice((PhidgetHandle)ch_I, channelInfo_I.isHubPortDevice);
	CheckError(prc, "Setting IsHubPortDevice", (PhidgetHandle *)&ch_I);
	prc = Phidget_setIsHubPortDevice((PhidgetHandle)ch_H, channelInfo_H.isHubPortDevice);
	CheckError(prc, "Setting IsHubPortDevice", (PhidgetHandle *)&ch_H);
	prc = Phidget_setIsHubPortDevice((PhidgetHandle)ch_B, channelInfo_B.isHubPortDevice);
	CheckError(prc, "Setting IsHubPortDevice", (PhidgetHandle *)&ch_B);

	prc = Phidget_setChannel((PhidgetHandle)ch_V, channelInfo_V.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_V);
	prc = Phidget_setChannel((PhidgetHandle)ch_CT, channelInfo_CT.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setChannel((PhidgetHandle)ch_I, channelInfo_I.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_I);
	prc = Phidget_setChannel((PhidgetHandle)ch_H, channelInfo_H.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_H);
	prc = Phidget_setChannel((PhidgetHandle)ch_P, channelInfo_P.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_P);
	prc = Phidget_setChannel((PhidgetHandle)ch_T, channelInfo_T.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_T);
	prc = Phidget_setChannel((PhidgetHandle)ch_R, channelInfo_R.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_R);
	prc = Phidget_setChannel((PhidgetHandle)ch_L, channelInfo_L.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_L);
	prc = Phidget_setChannel((PhidgetHandle)ch_B, channelInfo_B.channel);
	CheckError(prc, "Setting Channel", (PhidgetHandle *)&ch_B);

	/*
	if (channelInfo.netInfo.isRemote) {
		prc = Phidget_setIsRemote((PhidgetHandle)ch, channelInfo.netInfo.isRemote);
		CheckError(prc, "Setting IsRemote", (PhidgetHandle *)&ch);
		if (channelInfo.netInfo.serverDiscovery) {
			prc = PhidgetNet_enableServerDiscovery(PHIDGETSERVER_DEVICEREMOTE);
			CheckEnableServerDiscoveryError(prc, (PhidgetHandle *)&ch);
		}
		else {
			prc = PhidgetNet_addServer("Server", channelInfo.netInfo.hostname,
				channelInfo.netInfo.port, channelInfo.netInfo.password, 0);
			CheckError(prc, "Adding Server", (PhidgetHandle *)&ch);
		}
	}
	*/

	/*
	* Add event handlers before calling open so that no events are missed.
	*/

	printf("\n--------------------------------------\n");
	printf("\nSetting OnAttachHandler...\n");
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_V, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_V);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_CT, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_I, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_I);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_H, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_H);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_P, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_P);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_T, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_T);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_R, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_R);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_L, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_L);
	prc = Phidget_setOnAttachHandler((PhidgetHandle)ch_B, onAttachHandler, NULL);
	CheckError(prc, "Setting OnAttachHandler", (PhidgetHandle *)&ch_B);

	printf("Setting OnDetachHandler...\n");
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_V, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_V);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_CT, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_I, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_I);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_H, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_H);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_P, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_P);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_T, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_T);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_R, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_R);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_L, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_L);
	prc = Phidget_setOnDetachHandler((PhidgetHandle)ch_B, onDetachHandler, NULL);
	CheckError(prc, "Setting OnDetachHandler", (PhidgetHandle *)&ch_B);

	printf("Setting OnErrorHandler...\n");
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_V, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_V);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_CT, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_CT);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_I, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_I);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_H, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_H);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_P, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_P);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_T, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_T);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_R, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_R);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_L, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_L);
	prc = Phidget_setOnErrorHandler((PhidgetHandle)ch_B, onErrorHandler, NULL);
	CheckError(prc, "Setting OnErrorHandler", (PhidgetHandle *)&ch_B);


	//This call may be harmlessly removed
	//PrintEventDescriptions();

	/*
	* Open the channel with a timeout
	*/
	printf("Opening and Waiting for Attachment...\n");
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_V, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_V);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_CT, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_CT);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_I, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_I);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_H, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_H);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_P, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_P);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_T, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_T);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_R, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_R);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_L, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_L);
	prc = Phidget_openWaitForAttachment((PhidgetHandle)ch_B, 5000);
	CheckOpenError(prc, (PhidgetHandle *)&ch_B);

	/*
	* To find additional functionality not included in this example,
	* be sure to check the API for your device.
	*/
	Initialization();
	AskExecuteTime();
	fp = fopen("Information.csv", "w");

	while (clock() <= endclock)
	{
		//-------Time-------
		nowtime = time(NULL);
		timeinfo = localtime(&nowtime);
		nowclock = clock();

		if (flag_wait_display && (endclock - nowclock) % dataclock == 0)
		{
			flag_wait_display = false;

			if (flag_wait_change_Rotate && (endtime - nowtime) % _TIME_BASE_ROTATION == 0)
			{
				flag_wait_change_Rotate = false;
				flag_clockwise = !flag_clockwise;
			}
			else if (!flag_wait_change_Rotate && (endtime - nowtime) % _TIME_BASE_ROTATION == 0)
			{
				flag_wait_change_Rotate = false;
			}
			else
			{
				flag_wait_change_Rotate = true;
			}

			if (flag_wait_change_Actuator && (endtime - nowtime) % _TIME_BASE_ACTUATOR == 0)
			{
				flag_wait_change_Actuator = false;
				flag_act_push = !flag_act_push;
			}
			else if (!flag_wait_change_Actuator && (endtime - nowtime) % _TIME_BASE_ACTUATOR == 0)
			{
				flag_wait_change_Actuator = false;
			}
			else
			{
				flag_wait_change_Actuator = true;
			}

			if (flag_clockwise)
			{
				strcpy(status_servo, CW);
#if _TEST_MODE_ENABLE > 0
				prc = PhidgetRCServo_setTargetPosition(ch_R, 81);
				CheckError(prc, "Setting Target Position", (PhidgetHandle *)&ch_R);
#else
				prc = PhidgetRCServo_setTargetPosition(ch_R, 91);
				CheckError(prc, "Setting Target Position", (PhidgetHandle *)&ch_R);
#endif
			}
			else
			{
				strcpy(status_servo, CCW);
#if _TEST_MODE_ENABLE > 0
				prc = PhidgetRCServo_setTargetPosition(ch_R, 80);
				CheckError(prc, "Setting Target Position", (PhidgetHandle *)&ch_R);
#else
				prc = PhidgetRCServo_setTargetPosition(ch_R, 70);
				CheckError(prc, "Setting Target Position", (PhidgetHandle *)&ch_R);
#endif
			}

			if (flag_act_push)
			{
				strcpy(status_act, PUSH);
				prc = PhidgetDCMotor_setTargetVelocity(ch_L, 0.18);
				CheckError(prc, "Setting Velocity", (PhidgetHandle *)&ch_L);
			}
			else
			{
				strcpy(status_act, PULL);
				prc = PhidgetDCMotor_setTargetVelocity(ch_L, -0.18);
				CheckError(prc, "Setting Velocity", (PhidgetHandle *)&ch_L);
			}

			if (flag_wait_change_Relay && (endtime - nowtime) % _TIME_BASE_RELAY == 0)
			{
				flag_wait_change_Relay = false;
				flag_relay_off = !flag_relay_off;
			}
			else if (!flag_wait_change_Relay && (endtime - nowtime) % _TIME_BASE_RELAY == 0)
			{
				flag_wait_change_Relay = false;
			}
			else
			{
				flag_wait_change_Relay = true;
			}

			if (flag_relay_off)
			{
				strcpy(status_relay, ROFF);
				prc = PhidgetDigitalOutput_setDutyCycle(ch_B, 1.0);
				CheckError(prc, "Setting Duty Cycle", (PhidgetHandle *)&ch_B);
			}
			else
			{
				strcpy(status_relay, RON);
				prc = PhidgetDigitalOutput_setDutyCycle(ch_B, 0.0);
				CheckError(prc, "Setting Duty Cycle", (PhidgetHandle *)&ch_B);
			}

			//-------Voltage-------
#if _TEST_MODE_ENABLE > 0
			prc_Volt = PhidgetVoltageInput_getVoltage(ch_V, &valueVolt);
#else
			prc_Volt = PhidgetVoltageInput_getSensorValue(ch_V, &valueVolt);
			prc_Unit_V = PhidgetVoltageInput_getSensorUnit(ch_V, &sensorUnit_Volt);
#endif
			//-------CT-------
#if _TEST_MODE_ENABLE > 0
			prc_CT = PhidgetVoltageInput_getVoltage(ch_CT, &valueCT);
#else
			prc_CT = PhidgetVoltageInput_getSensorValue(ch_CT, &valueCT);
			prc_Unit_CT = PhidgetVoltageInput_getSensorUnit(ch_CT, &sensorUnit_CT);
#endif
			//-------Current-------
#if _TEST_MODE_ENABLE > 0
			prc_Curr = PhidgetVoltageRatioInput_getVoltageRatio(ch_I, &valueCurr);
#else
			prc_Curr = PhidgetVoltageRatioInput_getSensorValue(ch_I, &valueCurr);
			prc_Unit_I = PhidgetVoltageRatioInput_getSensorUnit(ch_I, &sensorUnit_Curr);
#endif
			//------Humidity-------
			prc_Hum = PhidgetVoltageRatioInput_getSensorValue(ch_H, &valueHum);
			prc_Unit_H = PhidgetVoltageRatioInput_getSensorUnit(ch_H, &sensorUnit_Hum);
			//-------Air Pressure-------
			prc_AP = PhidgetVoltageRatioInput_getSensorValue(ch_P, &valueAP);
			prc_Unit_P = PhidgetVoltageRatioInput_getSensorUnit(ch_P, &sensorUnit_AP);
			//------Temperature-------
			prc_Temp = PhidgetTemperatureSensor_getTemperature(ch_T, &valueTemp);

#if _TEST_MODE_ENABLE > 0
			if (prc_Volt == EPHIDGET_OK && prc_CT == EPHIDGET_OK && prc_Curr == EPHIDGET_OK && prc_Hum == EPHIDGET_OK
				&& prc_Unit_H == EPHIDGET_OK && prc_AP == EPHIDGET_OK && prc_Unit_P == EPHIDGET_OK)
#else
			if (prc_Volt == EPHIDGET_OK && prc_CT == EPHIDGET_OK && prc_Curr == EPHIDGET_OK && prc_Hum == EPHIDGET_OK
				&& prc_Unit_V == EPHIDGET_OK && prc_Unit_CT == EPHIDGET_OK && prc_Unit_I == EPHIDGET_OK
				&& prc_Unit_H == EPHIDGET_OK && prc_AP == EPHIDGET_OK && prc_Unit_P == EPHIDGET_OK)
#endif
			{
#if _TEST_MODE_ENABLE > 0
				printf("Clock = %ld ", nowclock);
				printf("Cycle %d: %.2f V %.2f A %.2f A %.2f %sH %.2f %s %.2f degC %s %s %s %s", cnt_data, valueVolt, valueCT, valueCurr, valueHum, sensorUnit_Hum.symbol, valueAP, sensorUnit_AP.symbol, valueTemp, status_servo, status_act, status_relay, asctime(timeinfo));
				fprintf(fp, "Cycle %d, %.2f, V, %.2f, A, %.2f, A, %.2f, %sH, %.2f, %s, %.2f, degC, %s, %s, %s, %s", cnt_data, valueVolt, valueCT, valueCurr, valueHum, sensorUnit_Hum.symbol, valueAP, sensorUnit_AP.symbol, valueTemp, status_servo, status_act, status_relay, asctime(timeinfo));
#else
				printf("Clock = %ld ", nowclock);
				printf("Cycle %d: %.2f %s %.2f %s %.2f %s %.2f %sH %.2f %s %.2f degC %s %s %s %s", cnt_data, valueVolt, sensorUnit_Volt.symbol, valueCT, sensorUnit_CT.symbol, valueCurr, sensorUnit_Curr.symbol, valueHum, sensorUnit_Hum.symbol, valueAP, sensorUnit_AP.symbol, valueTemp, status_servo, status_act, status_relay, asctime(timeinfo));
				fprintf(fp, "Cycle %d, %.2f, %s, %.2f, %s, %.2f, %s, %.2f, %sH, %.2f, %s, %.2f ,degC , %s, %s, %s, %s", cnt_data, valueVolt, sensorUnit_Volt.symbol, valueCT, sensorUnit_CT.symbol, valueCurr, sensorUnit_Curr.symbol, valueHum, sensorUnit_Hum.symbol, valueAP, sensorUnit_AP.symbol, valueTemp, status_servo, status_act, status_relay, asctime(timeinfo));
#endif
				cnt_data++;
			}
		}
		else if (!flag_wait_display && (endclock - nowclock) % dataclock == 0)
		{
			flag_wait_display = false;
		}
		else
		{
			flag_wait_display = true;
		}
	}

	fclose(fp);

	/*
	* Perform clean up and exit
	*/

	//clear the VoltageRatioChange event handler
	prc = PhidgetVoltageInput_setOnVoltageChangeHandler(ch_V, NULL, NULL);
	CheckError(prc, "Clearing OnVoltageRatioChangeHandler", (PhidgetHandle *)&ch_V);
	prc = PhidgetVoltageInput_setOnVoltageChangeHandler(ch_CT, NULL, NULL);
	CheckError(prc, "Clearing OnVoltageRatioChangeHandler", (PhidgetHandle *)&ch_CT);
	prc = PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch_I, NULL, NULL);
	CheckError(prc, "Clearing OnVoltageRatioChangeHandler", (PhidgetHandle *)&ch_I);
	prc = PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch_H, NULL, NULL);
	CheckError(prc, "Clearing OnVoltageRatioChangeHandler", (PhidgetHandle *)&ch_H);
	prc = PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch_P, NULL, NULL);
	CheckError(prc, "Clearing OnVoltageRatioChangeHandler", (PhidgetHandle *)&ch_P);

	//clear the SensorChange event handler
	prc = PhidgetVoltageInput_setOnSensorChangeHandler(ch_V, NULL, NULL);
	CheckError(prc, "Clearing OnSensorChangeHandler", (PhidgetHandle *)&ch_V);
	prc = PhidgetVoltageInput_setOnSensorChangeHandler(ch_CT, NULL, NULL);
	CheckError(prc, "Clearing OnSensorChangeHandler", (PhidgetHandle *)&ch_CT);
	prc = PhidgetVoltageRatioInput_setOnSensorChangeHandler(ch_I, NULL, NULL);
	CheckError(prc, "Clearing OnSensorChangeHandler", (PhidgetHandle *)&ch_I);
	prc = PhidgetVoltageRatioInput_setOnSensorChangeHandler(ch_H, NULL, NULL);
	CheckError(prc, "Clearing OnSensorChangeHandler", (PhidgetHandle *)&ch_H);
	prc = PhidgetVoltageRatioInput_setOnSensorChangeHandler(ch_P, NULL, NULL);
	CheckError(prc, "Clearing OnSensorChangeHandler", (PhidgetHandle *)&ch_P);

	//clear the TemperatureChange event handler
	prc = PhidgetTemperatureSensor_setOnTemperatureChangeHandler(ch_T, NULL, NULL);
	CheckError(prc, "Clearing OnTemperatureChangeHandler", (PhidgetHandle *)&ch_T);

	//clear the TargetPositionReached event handler
	prc = PhidgetRCServo_setOnTargetPositionReachedHandler(ch_R, NULL, NULL);
	CheckError(prc, "Clearing OnTargetPositionReachedHandler", (PhidgetHandle *)&ch_R);

	//clear the VelocityUpdate event handler
	prc = PhidgetDCMotor_setOnVelocityUpdateHandler(ch_L, NULL, NULL);
	CheckError(prc, "Clearing OnVelocityUpdateHandler", (PhidgetHandle *)&ch_L);

	printf("\nDone Sampling...\n");

	printf("Cleaning up...\n");
	prc = Phidget_close((PhidgetHandle)ch_V);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_V);
	prc = Phidget_close((PhidgetHandle)ch_CT);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_CT);
	prc = Phidget_close((PhidgetHandle)ch_I);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_I);
	prc = Phidget_close((PhidgetHandle)ch_H);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_H);
	prc = Phidget_close((PhidgetHandle)ch_P);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_P);
	prc = Phidget_close((PhidgetHandle)ch_T);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_T);
	prc = Phidget_close((PhidgetHandle)ch_R);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_R);
	prc = Phidget_close((PhidgetHandle)ch_L);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_L);
	prc = Phidget_close((PhidgetHandle)ch_B);
	CheckError(prc, "Closing Channel", (PhidgetHandle *)&ch_B);

	prc = PhidgetVoltageInput_delete(&ch_V);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_V);
	prc = PhidgetVoltageInput_delete(&ch_CT);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_CT);
	prc = PhidgetVoltageRatioInput_delete(&ch_I);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_I);
	prc = PhidgetVoltageRatioInput_delete(&ch_H);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_H);
	prc = PhidgetVoltageRatioInput_delete(&ch_P);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_P);
	prc = PhidgetTemperatureSensor_delete(&ch_T);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_T);
	prc = PhidgetRCServo_delete(&ch_R);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_R);
	prc = PhidgetDCMotor_delete(&ch_L);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_L);
	prc = PhidgetDigitalOutput_delete(&ch_B);
	CheckError(prc, "Deleting Channel", (PhidgetHandle *)&ch_B);

	tarTime = localtime(&endtime);
	//printf("\nStart Time = %lld; End Time = %lld\n", starttime, endtime);
	printf("\nTotal Executing Time = %llds\n", exetime);
	printf("\nStart Time = %s", ctime(&starttime));
	printf("  End Time = %s\n", asctime(tarTime));

	printf("\nData Interval Time = %ldms\n", dataclock);

	printf("\nTotal Executing Clock = %ldms\n", endclock - startclock);
	printf("\nStart Clock = %ldms", startclock);
	printf("\n  End Clock = %ldms\n", endclock);

	printf("\nPress ENTER to end program.\n");
	getchar();
	getchar();

	return 0;
}