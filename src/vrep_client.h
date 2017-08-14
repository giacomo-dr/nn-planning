// vrep_client.h
// Author: Giacomo Del Rio
// Date: 12 Apr 2017

#ifndef VREP_CLIENT_H
#define VREP_CLIENT_H

#include <iostream>
#include <string>
extern "C" {
    #include "extApi.h"
}
#include "geometry.h"


class VRepClient {

public:
    VRepClient( const std::string address, int port );
    ~VRepClient();
    void connect();
    void disconnect();
    bool is_connected();
    int get_handle( std::string name );
    void sleep( long ms );
    long get_last_cmd_time();
    void set_joint_target_velocity( int joint_handle, double velocity );
    void set_joint_target_position( int joint_handle, double position );
    void set_object_position( int object_handle, double x, double y, double z );
    void set_object_orientation( int object_handle, double a, double b, double g );
    void set_dynamic_object_pose( int object_handle, Point3D pos, Point3D orient );
    void register_for_position( int object_handle, int delay_ms );
    void register_for_orientation( int object_handle, int delay_ms );
    void unregister_for_position( int object_handle );
    void unregister_for_orientation( int object_handle );
    Point3D get_object_position( int object_handle );
    Point3D get_object_orientation( int object_handle );
    int create_heightfield_shape( int x_points, int y_points, double x_meters,
                                  double z_meters, const unsigned char* values );
    void start_simulation();
    void stop_simulation();
    void pause_simulation();

private:
    std::string server_address;
    int server_port;
    int client_id;
};


#endif //VREP_CLIENT_H
