#include <LibDS.h>
#include <iostream>

int main(int argc, char *argv[])
{
    DS_Init();
    DS_SetCustomRobotAddress("127.0.0.1");

    DS_Protocol proto = DS_GetProtocolFRC_2020();
    DS_ConfigureProtocol(&proto);

    DS_SetControlMode(DS_CONTROL_TELEOPERATED);
    DS_SetRobotEnabled(true);

    DS_Close();
    return 0;
}