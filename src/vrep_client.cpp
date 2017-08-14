// vrep_client.cpp
// Author: Giacomo Del Rio
// Date: 12 Apr 2017

#include "vrep_client.h"

VRepClient::VRepClient( const std::string address, int port ){
    this->server_address = address;
    this->server_port = port;
    client_id = -1;
}

VRepClient::~VRepClient(){
    if( client_id != -1 )
        disconnect();
}

void VRepClient::connect(){
    client_id = simxStart( (simxChar*)server_address.c_str(),
                           server_port, true, true, 2000, 5 );
}

void VRepClient::disconnect(){
    if( client_id != -1 )
        simxFinish( client_id );
    client_id = -1;
}

bool VRepClient::is_connected(){
    if( client_id != -1 ){
        if( simxGetConnectionId(client_id) == -1 )
            client_id = -1; // Disconnected
    }
    return client_id != -1;
}

int VRepClient::get_handle( std::string name ){
    int handle;
    simxGetObjectHandle( client_id, (simxChar*)name.c_str(),
                         &handle, simx_opmode_blocking );
    return handle;
}

void VRepClient::sleep( long ms ) {
    extApi_sleepMs( ms );
}

long VRepClient::get_last_cmd_time(){
    return simxGetLastCmdTime( client_id );
}

void VRepClient::set_joint_target_velocity( int joint_handle, double velocity ){
    simxSetJointTargetVelocity( client_id, joint_handle,
                                velocity, simx_opmode_oneshot );
}

void VRepClient::set_joint_target_position( int joint_handle, double position ){
    simxSetJointTargetPosition( client_id, joint_handle,
                                position, simx_opmode_oneshot );
}

void VRepClient::set_object_position( int object_handle, double x, double y, double z ){
    simxFloat pos[] = { (simxFloat)x, (simxFloat)y, (simxFloat)z };
    simxSetObjectPosition( client_id, object_handle, -1, pos, simx_opmode_oneshot );
}

void VRepClient::set_object_orientation( int object_handle, double a, double b, double g ){
    simxFloat pos[] = { (simxFloat)a, (simxFloat)b, (simxFloat)g };
    simxSetObjectOrientation( client_id, object_handle, -1, pos, simx_opmode_oneshot );
}

void VRepClient::set_dynamic_object_pose( int object_handle, Point3D pos, Point3D orient ){
    simxFloat pose[] = { (simxFloat)pos(0), (simxFloat)pos(1), (simxFloat)pos(2),
                         (simxFloat)orient(0), (simxFloat)orient(1), (simxFloat)orient(2)};
    simxCallScriptFunction(
            client_id, "utils_script", sim_scripttype_childscript,
            "setPose_function", 1, &object_handle, 6, pose,
            0, NULL, 0, NULL, NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, simx_opmode_blocking );
}

void VRepClient::register_for_position( int object_handle, int delay_ms ){
    assert( delay_ms == 0 ); // >0 doesn't work
    float buf[3];
    simxGetObjectPosition( client_id, object_handle, -1,
                           buf, simx_opmode_streaming + delay_ms );
}

void VRepClient::register_for_orientation( int object_handle, int delay_ms ){
    assert( delay_ms == 0 ); // >0 doesn't work
    float buf[3];
    simxGetObjectOrientation( client_id, object_handle, -1,
                              buf, simx_opmode_streaming + delay_ms );
}

void VRepClient::unregister_for_position( int object_handle ){
    float buf[3];
    simxGetObjectPosition( client_id, object_handle, -1,
                           buf, simx_opmode_discontinue );
}

void VRepClient::unregister_for_orientation( int object_handle ){
    float buf[3];
    simxGetObjectOrientation( client_id, object_handle, -1,
                              buf, simx_opmode_discontinue );
}

Point3D VRepClient::get_object_position( int object_handle ){
    float buf[3];
    simxGetObjectPosition( client_id, object_handle, -1, buf, simx_opmode_buffer );
    return Point3D( buf[0], buf[1], buf[2] );
}

Point3D VRepClient::get_object_orientation( int object_handle ){
    float buf[3];
    simxGetObjectOrientation( client_id, object_handle, -1, buf, simx_opmode_buffer );
    return Point3D( buf[0], buf[1], buf[2] );
}

int VRepClient::create_heightfield_shape( int x_points, int y_points, double x_meters,
                                          double z_meters, const unsigned char* values ){
    int xy_points[2] = { x_points, y_points };
    simxFloat xz_meters[2] = { (simxFloat)x_meters, (simxFloat)z_meters };
    int ret_int_count;
    simxInt* hf_handle;
    int result = simxCallScriptFunction(
            client_id, "utils_script", sim_scripttype_childscript,
            "createHeightField_function", 2, xy_points, 2, xz_meters,
            0, NULL, x_points * y_points, values, &ret_int_count, &hf_handle, NULL, NULL,
            NULL, NULL, NULL, NULL, simx_opmode_blocking );
    return result == simx_return_ok ? hf_handle[0] : -1;
}

void VRepClient::start_simulation(){
    simxStartSimulation( client_id, simx_opmode_oneshot );
}

void VRepClient::stop_simulation(){
    simxStopSimulation( client_id, simx_opmode_oneshot );
}

void VRepClient::pause_simulation(){
    simxPauseSimulation( client_id, simx_opmode_oneshot );
}

